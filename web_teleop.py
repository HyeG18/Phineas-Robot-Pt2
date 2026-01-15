from picamera2 import Picamera2
from fastapi import FastAPI, Response, Request
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from gpiozero import Motor, Device
from gpiozero.pins.lgpio import LGPIOFactory
import time
import cv2
import numpy as np
import threading
import serial
import pynmea2
import sys
import math
import struct 
from smbus2 import SMBus 
# import tflite_runtime.interpreter as tflite # IA COMENTADA
from typing import Optional, List
import asyncio
from pydantic import BaseModel
from collections import deque

# --- CONFIGURACIÓN DEL SERVIDOR ---
app = FastAPI()

app.mount("/static", StaticFiles(directory="static"), name="static")
templates = Jinja2Templates(directory="templates")

# --- CONSTANTES VISIÓN ---
JPEG_QUALITY = 60       
FRAME_PROCESS_DELAY = 0.05 
FRAME_WIDTH = 640       
FRAME_CENTER_X = FRAME_WIDTH // 2 

# Ajustes de Control
KP_STEER = 0.004        
MAX_STEER_ADJUSTMENT = 0.4 
TARGET_BOX_WIDTH = 250  
WIDTH_TOLERANCE = 50    
MIN_MOVING_SPEED = 0.60 
DECEL_BOX_WIDTH = 150   

# Configuración GPS
GPS_SERIAL_PORT = '/dev/ttyAMA0'
GPS_BAUD_RATE = 38400 

# Configuración Brújula
COMPASS_ADDRESS = 0x0D
COMPASS_PORT = 1

# ==========================================
# 🛠️ ZONA DE CALIBRACIÓN FÍSICA (TUS DATOS)
# ==========================================

# 1. CENTRADO DEL SENSOR (Valores de tu "Baile")
# Esto hace que el círculo magnético sea perfecto.
HARD_IRON_X = -230.0
HARD_IRON_Y = 5211.0

# 2. ROTACIÓN DEL SENSOR (Tu Norte Real)
# Si apuntando al Norte marcaba 242°, restamos 242 para que marque 0°.
COMPASS_FINAL_OFFSET = -62.0 

# 3. BALANCEO DE RUEDAS (Trim)
# Si la derecha va muy rápido, le bajamos la potencia (ej: 0.90 es 90% de potencia).
TRIM_LEFT = 1.0   
TRIM_RIGHT = 1.0  # <--- BAJADO para igualar a la izquierda

# ==========================================

# --- AJUSTES FINOS DE NAVEGACIÓN ---
WAYPOINT_TOLERANCE_METERS = 3.0  
NAV_SPEED = 0.55                 
NAV_TURN_SPEED = 0.35            
HEADING_TOLERANCE_DEG = 45.0     
DEADBAND_DEG = 8.0               

# --- CLASE BRÚJULA PROFESIONAL ---
class Compass:
    def __init__(self):
        self.bus = SMBus(COMPASS_PORT)
        self.address = COMPASS_ADDRESS
        # Cargamos tus valores fijos
        self.x_offset = HARD_IRON_X
        self.y_offset = HARD_IRON_Y
        
        try:
            self.bus.write_byte_data(self.address, 0x09, 0x01) 
            self.bus.write_byte_data(self.address, 0x0B, 0x01) 
            print(f"✅ Brújula Calibrada (Hard Iron: {self.x_offset}, {self.y_offset})")
        except Exception as e:
            print(f"⚠️ Error iniciando Brújula: {e}")

    def get_heading(self):
        try:
            with SMBus(COMPASS_PORT) as bus:
                data = bus.read_i2c_block_data(self.address, 0x00, 6)
                
                # 1. Leer datos crudos
                x_raw = struct.unpack('<h', bytes(data[0:2]))[0]
                y_raw = struct.unpack('<h', bytes(data[2:4]))[0]
                
                # 2. Aplicar Calibración Hard Iron (El Baile)
                x = x_raw - self.x_offset
                y = y_raw - self.y_offset
                
                # 3. Calcular Ángulo
                heading_rad = math.atan2(y, x)
                heading_deg = math.degrees(heading_rad)
                
                # 4. Aplicar Corrección de Rotación (-242 grados)
                heading_deg += COMPASS_FINAL_OFFSET

                # 5. Normalizar 0-360
                if heading_deg < 0: heading_deg += 360
                elif heading_deg > 360: heading_deg -= 360
                
                return heading_deg
        except: return None
        
    def calibrate_north(self):
        # Ya no hace nada porque usamos valores fijos y precisos
        return True 

# --- FUNCIONES MATEMÁTICAS ---
def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000 
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    return (initial_bearing + 360) % 360

def get_heading_error(current_heading, target_bearing):
    error = target_bearing - current_heading
    if error > 180: error -= 360
    if error < -180: error += 360
    return error

# --- VISION PROCESSOR (SOLO CÁMARA) ---
class VisionProcessor:
    def __init__(self, picam2):
        self.camera = picam2
        self.running = True
        self.lock = threading.Lock()
        self.latest_frame = None 
        self.latest_detection = None
        self.thread = threading.Thread(target=self.process_loop, daemon=True)
        self.thread.start()

    def process_loop(self):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY] 
        while self.running:
            frame_yuv = self.camera.capture_array()
            if frame_yuv is None:
                time.sleep(0.01)
                continue
            frame_bgr = cv2.cvtColor(frame_yuv, cv2.COLOR_YUV2BGR_I420)
            frame_display = cv2.rotate(frame_bgr, cv2.ROTATE_180)
            ret, buffer = cv2.imencode('.jpg', frame_display, encode_param)
            with self.lock:
                self.latest_frame = buffer.tobytes()
                self.latest_detection = None
            time.sleep(FRAME_PROCESS_DELAY) 

    def get_latest_frame(self):
        with self.lock: return self.latest_frame
    def get_latest_detection(self):
        with self.lock: return self.latest_detection
    def stop(self):
        self.running = False
        self.thread.join()

# --- ROBOT CONTROLLER ---
class RobotController:
    def __init__(self, motorA, motorB, picam2):
        self.motorA = motorA 
        self.motorB = motorB 
        self.camera = picam2
        
        self.compass = Compass() 
        self.running_compass = True 
        
        self.current_mode = "manual"
        self.mode_lock = threading.Lock() 
        self.vision_task = None 
        self.gps_task = None
        self.compass_task = None 
        
        self.manual_base_speed = 0.6   
        self.manual_turn_speed = 0.8  
        
        self.gps_data = {
            "latitude": "N/A", "longitude": "N/A",
            "lat_raw": 0.0, "lng_raw": 0.0,
            "course": 0.0, 
            "speed_kmh": 0.0,
            "satellites": "N/A", "timestamp": "N/A",
            "fix_status": "Desactivado",
            "nav_active": False,
            "nav_wp_index": 0,
            "nav_dist_m": 0.0,
            "nav_target_bearing": 0.0,
            "nav_heading_error": 0.0
        }
        self.gps_lock = threading.Lock() 
        self.running_gps = False 
        self.gps_history_lat = deque(maxlen=5) 
        self.gps_history_lng = deque(maxlen=5)
        
        self.waypoints: List[dict] = [] 
        self.current_waypoint_index: int = 0

    def set_motor_speeds(self, left_speed, right_speed):
        # --- APLICAMOS EL TRIM PARA CORREGIR DESVIACIÓN ---
        if abs(left_speed) > 0.1 and abs(right_speed) > 0.1:
            left_speed *= TRIM_LEFT
            right_speed *= TRIM_RIGHT
            
        left_speed = np.clip(left_speed, -1.0, 1.0)
        right_speed = np.clip(right_speed, -1.0, 1.0)
        self.motorA.value = left_speed
        self.motorB.value = right_speed

    def move(self, direction):
        speed = self.manual_base_speed 
        turn = self.manual_turn_speed 
        if direction == 'forward': self.set_motor_speeds(speed, speed)
        elif direction == 'backward': self.set_motor_speeds(-speed, -speed)
        elif direction == 'left': self.set_motor_speeds(-turn, turn) 
        elif direction == 'right': self.set_motor_speeds(turn, -turn) 
        elif direction == 'stop': self.set_motor_speeds(0, 0)

    def get_mode(self):
        with self.mode_lock: return self.current_mode
    def set_mode(self, mode):
        with self.mode_lock: self.current_mode = mode
    def get_gps_data(self):
        with self.gps_lock: return self.gps_data.copy()

# --- HARDWARE INIT ---
Device.pin_factory = LGPIOFactory()
motorA = Motor(forward=23, backward=24, enable=25, pwm=True)
motorB = Motor(forward=17, backward=22, enable=27, pwm=True)

picam2 = Picamera2()
config = picam2.create_video_configuration(main={"size": (640, 480), "format": "YUV420"})
picam2.configure(config)
try: picam2.start(); time.sleep(1) 
except: sys.exit(1)

ROBOT = RobotController(motorA, motorB, picam2) 
VISION_PROCESSOR = VisionProcessor(picam2) 

# --- TASKS ---
async def generate_frames():
    while True:
        frame_bytes = VISION_PROCESSOR.get_latest_frame()
        if frame_bytes is None:
            await asyncio.sleep(0.05)
            continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        await asyncio.sleep(0.001) 

async def compass_loop_async(robot):
    print("🧭 Iniciando Loop de Brújula...")
    while robot.running_compass: 
        try:
            heading = robot.compass.get_heading()
            if heading is not None:
                with robot.gps_lock:
                    robot.gps_data["course"] = heading
            await asyncio.sleep(0.1) 
        except Exception as e:
            print(f"Error Loop Brújula: {e}")
            await asyncio.sleep(1)

# --- GPS TURBO ---
async def gps_reader_loop_async(robot: RobotController):
    ser = None
    try:
        ser = serial.Serial(GPS_SERIAL_PORT, baudrate=GPS_BAUD_RATE, timeout=1)
        print("✅ GPS Iniciado Correctamente")
    except Exception as e:
        print(f"❌ GPS Error de Hardware: {e}")
        robot.running_gps = False
        return 

    loop = asyncio.get_event_loop()
    ser.reset_input_buffer()

    while robot.running_gps:
        try:
            if ser.in_waiting > 0:
                line = await loop.run_in_executor(None, ser.readline)
                try:
                    decoded_line = line.decode('ascii', errors='replace').strip()
                    if decoded_line.startswith(('$GNGGA', '$GPGGA', '$GNRMC', '$GPRMC')):
                        msg = pynmea2.parse(decoded_line)
                        status_update = None
                        sats_update = 0
                        if 'GGA' in decoded_line:
                            sats_update = int(msg.num_sats) if msg.num_sats else 0
                            if int(msg.gps_qual) > 0: status_update = "FIX 3D"
                            else: status_update = "Buscando..."
                        elif 'RMC' in decoded_line:
                            if msg.status == 'A': status_update = "FIX Válido"
                            else: status_update = "Buscando..."

                        if status_update or sats_update > 0:
                            with robot.gps_lock:
                                if status_update: robot.gps_data["fix_status"] = status_update
                                if sats_update > 0: robot.gps_data["satellites"] = sats_update

                        if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                            try:
                                lat_val = float(msg.latitude)
                                lng_val = float(msg.longitude)
                                lat_dir = str(msg.lat_dir).strip().upper()
                                if lat_dir == 'S': lat_val = -abs(lat_val)
                                else: lat_val = abs(lat_val)
                                lng_val = -abs(lng_val)

                                if lat_val != 0.0 and lng_val != 0.0:
                                    with robot.gps_lock:
                                        robot.gps_history_lat.append(lat_val)
                                        robot.gps_history_lng.append(lng_val)
                                        avg_lat = sum(robot.gps_history_lat) / len(robot.gps_history_lat)
                                        avg_lng = sum(robot.gps_history_lng) / len(robot.gps_history_lng)
                                        robot.gps_data["lat_raw"] = avg_lat
                                        robot.gps_data["lng_raw"] = avg_lng
                                        robot.gps_data["latitude"] = f"{avg_lat:.6f}"
                                        robot.gps_data["longitude"] = f"{avg_lng:.6f}"
                                        if 'RMC' in decoded_line:
                                            robot.gps_data["speed_kmh"] = float(msg.spd_over_grnd) * 1.852 if msg.spd_over_grnd else 0.0
                            except ValueError: pass 
                except pynmea2.ParseError: pass
            else:
                await asyncio.sleep(0.01)
        except Exception as e:
            print(f"⚠️ Error lectura GPS: {e}")
            await asyncio.sleep(1)
    if ser: ser.close()
    
async def perrito_mode_loop_async(robot): pass
async def bottle_search_loop_async(robot): pass

async def navigation_mode_loop_async(robot: RobotController):
    print("🚀 Iniciando Navegación Autónoma SUAVE...")
    with robot.gps_lock: robot.gps_data["nav_active"] = True

    try:
        while robot.get_mode() == "navigation":
            if not robot.waypoints:
                robot.set_motor_speeds(0, 0)
                await asyncio.sleep(2)
                continue
            
            if robot.current_waypoint_index >= len(robot.waypoints):
                print("🏁 ¡Misión Completada!")
                robot.set_mode("manual") 
                break

            gps = robot.get_gps_data()
            curr_lat = gps['lat_raw']
            curr_lng = gps['lng_raw']
            curr_heading = gps['course']

            if curr_lat == 0.0 or curr_lng == 0.0:
                robot.set_motor_speeds(0, 0)
                await asyncio.sleep(1)
                continue

            target = robot.waypoints[robot.current_waypoint_index]
            target_lat, target_lng = target['lat'], target['lng']

            distance = haversine_distance(curr_lat, curr_lng, target_lat, target_lng)
            target_bearing = calculate_bearing(curr_lat, curr_lng, target_lat, target_lng)
            heading_error = get_heading_error(curr_heading, target_bearing)

            with robot.gps_lock:
                robot.gps_data["nav_wp_index"] = robot.current_waypoint_index + 1
                robot.gps_data["nav_dist_m"] = round(distance, 1)
                robot.gps_data["nav_target_bearing"] = round(target_bearing, 0)
                robot.gps_data["nav_heading_error"] = round(heading_error, 0)

            print(f"WP {robot.current_waypoint_index+1} | Dist: {distance:.1f}m | HEAD: {curr_heading:.0f}° | Err: {heading_error:.0f}")

            if distance < WAYPOINT_TOLERANCE_METERS:
                print(f"✅ Llegada al WP {robot.current_waypoint_index+1}")
                robot.set_motor_speeds(0, 0)
                await asyncio.sleep(1) 
                robot.current_waypoint_index += 1
                continue

            # --- CONTROL ZONA MUERTA + INVERTIDO ---
            if abs(heading_error) < DEADBAND_DEG:
                robot.set_motor_speeds(NAV_SPEED, NAV_SPEED)
            
            elif abs(heading_error) > HEADING_TOLERANCE_DEG:
                if heading_error > 0:
                    robot.set_motor_speeds(-NAV_TURN_SPEED, NAV_TURN_SPEED) 
                else:
                    robot.set_motor_speeds(NAV_TURN_SPEED, -NAV_TURN_SPEED) 
            else:
                correction = heading_error * 0.002 
                correction = max(min(correction, 0.2), -0.2)
                left_motor = NAV_SPEED - correction
                right_motor = NAV_SPEED + correction
                robot.set_motor_speeds(left_motor, right_motor)

            await asyncio.sleep(0.1) 

    except asyncio.CancelledError:
        print("🛑 Navegación cancelada.")
    except Exception as e:
        print(f"❌ Error crítico en navegación: {e}")
    finally:
        robot.set_motor_speeds(0,0)
        with robot.gps_lock: robot.gps_data["nav_active"] = False

# --- ROUTES ---
class Waypoint(BaseModel):
    lat: float
    lng: float
class WaypointsPayload(BaseModel):
    waypoints: List[Waypoint]

@app.get("/", response_class=HTMLResponse)
async def index(request: Request): return templates.TemplateResponse("index.html", {"request": request})

@app.get("/video_feed")
async def video_feed(): return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/gps_data")
async def gps_data(): return JSONResponse(ROBOT.get_gps_data())

@app.post("/waypoints")
async def receive_waypoints(payload: WaypointsPayload):
    ROBOT.waypoints = [wp.dict() for wp in payload.waypoints]
    ROBOT.current_waypoint_index = 0
    return JSONResponse({"status": "OK", "count": len(ROBOT.waypoints)})

@app.get("/mode/{new_mode}")
async def set_mode(new_mode: str):
    if new_mode == "gps":
        if ROBOT.running_gps:
            ROBOT.running_gps = False
            if ROBOT.gps_task: ROBOT.gps_task.cancel()
            with ROBOT.gps_lock: ROBOT.gps_data["fix_status"] = "Desactivado"
            return JSONResponse({"status": "GPS OFF"})
        else:
            ROBOT.running_gps = True
            with ROBOT.gps_lock: ROBOT.gps_data["fix_status"] = "Iniciando..." 
            ROBOT.gps_task = asyncio.create_task(gps_reader_loop_async(ROBOT))
            return JSONResponse({"status": "GPS ON"})

    current_mode = ROBOT.get_mode()
    ROBOT.set_mode("manual")
    ROBOT.set_motor_speeds(0,0)
    if ROBOT.vision_task: ROBOT.vision_task.cancel()
    
    if new_mode == "navigation":
        if not ROBOT.running_gps: return JSONResponse({"status": "Error: GPS Required"}, status_code=400)
        ROBOT.set_mode("navigation")
        ROBOT.vision_task = asyncio.create_task(navigation_mode_loop_async(ROBOT))
    elif new_mode == "manual":
        ROBOT.set_mode("manual")

    return JSONResponse({"status": f"Mode {new_mode} set"})

@app.get("/move/{direction}")
async def move_robot_api(direction: str):
    if ROBOT.get_mode() == "manual":
        ROBOT.move(direction)
        return JSONResponse({"status": "OK"})
    return JSONResponse({"status": "Busy"}, status_code=403)

@app.get("/calibrate_compass")
async def calibrate_compass_api():
    # La calibración dinámica ya no es necesaria con los Hard Iron values fijos
    return JSONResponse({"status": "OK", "offset": "Fixed via Hard Iron"})

@app.on_event("startup")
async def startup_event():
    ROBOT.compass_task = asyncio.create_task(compass_loop_async(ROBOT))

@app.on_event("shutdown")
async def shutdown():
    ROBOT.set_motor_speeds(0,0)
    ROBOT.running_compass = False 
    if ROBOT.vision_task: ROBOT.vision_task.cancel()
    if ROBOT.gps_task: ROBOT.gps_task.cancel()
    if ROBOT.compass_task: ROBOT.compass_task.cancel()
    VISION_PROCESSOR.stop()
    try: picam2.stop()
    except: pass