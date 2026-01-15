/**
 * Phineas Prime Control Logic
 * Versión: Telemetry + Compass Calibration
 */

// --- ELEMENTOS DOM (Interfaz) ---
const MODE_STATUS_DISPLAY = document.getElementById('mode-status');
const GPS_STATUS_DISPLAY = document.getElementById('gps-status-display');
const GPS_BUTTON = document.getElementById('gps-mode-btn');

// Datos GPS Básicos
const GPS_TXT_LAT = document.getElementById('gps-latitude');
const GPS_TXT_LNG = document.getElementById('gps-longitude');
const GPS_TXT_SAT = document.getElementById('gps-satellites');

// Botones Mapa
const BTN_CLEAR_MAP = document.getElementById('clear-map-btn');
const BTN_SEND_MISSION = document.getElementById('send-mission-btn');

// Botón de Calibración (NUEVO)
const BTN_CALIBRATE = document.getElementById('btn-calibrar');

// Panel Telemetría (Navegación)
const NAV_PANEL = document.getElementById('nav-telemetry');
const NAV_TXT_WP = document.getElementById('nav-wp-idx');
const NAV_TXT_DIST = document.getElementById('nav-dist');
const NAV_TXT_CURR_HEAD = document.getElementById('nav-current-head');
const NAV_TXT_TARG_HEAD = document.getElementById('nav-target-head');
const NAV_TXT_ERR = document.getElementById('nav-error');

// Referencias a la Brújula Visual
const COMPASS_ARROW = document.getElementById('compass-arrow');
const COMPASS_TXT = document.getElementById('compass-text-val');

// --- VARIABLES DE ESTADO ---
let currentMode = 'manual';
let isGpsActive = false;
let hasInitialFix = false; // Control para el zoom automático

// --- VARIABLES DEL MAPA ---
let map = null;
let robotMarker = null;
let waypoints = []; 
let waypointMarkers = []; 
let pathPolyline = null; 

// ==========================
// 1. INICIALIZACIÓN DEL MAPA
// ==========================
function initMap() {
    console.log("📍 Intentando iniciar mapa...");

    if (!document.getElementById('robot-map')) return;

    if (typeof L === 'undefined') {
        console.error("❌ ERROR: Leaflet no cargó. Revisa tu conexión a internet para descargar el mapa.");
        return;
    }

    try {
        // Inicializamos el mapa centrado en 0,0 (Vista Global)
        map = L.map('robot-map').setView([0, 0], 2);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 19
        }).addTo(map);

        // Marcador del Robot (Círculo Azul)
        robotMarker = L.circleMarker([0, 0], {
            color: 'white',       
            weight: 3,
            fillColor: '#007BFF', 
            fillOpacity: 1,
            radius: 8
        }).addTo(map);

        // Capa para la línea de ruta
        pathPolyline = L.polyline([], { color: 'red' }).addTo(map);

        // Evento click: Poner Waypoints
        map.on('click', function(e) {
            addWaypoint(e.latlng);
        });

        // Corrección de renderizado
        setTimeout(() => { map.invalidateSize(); }, 500);
        console.log("✅ Mapa iniciado correctamente.");

    } catch (e) {
        console.error("❌ Excepción al iniciar mapa:", e);
    }
}

function addWaypoint(latlng) {
    waypoints.push({ lat: latlng.lat, lng: latlng.lng });
    let index = waypoints.length;
    
    // Icono numérico
    let wpIcon = L.divIcon({
        className: 'waypoint-icon',
        html: `<div style="background:white; border:2px solid black; border-radius:50%; width:20px; height:20px; text-align:center; line-height:20px; font-weight:bold;">${index}</div>`,
        iconSize: [24, 24],
        iconAnchor: [12, 12]
    });
    
    let marker = L.marker(latlng, {icon: wpIcon}).addTo(map);
    waypointMarkers.push(marker);
    drawPath();
}

function drawPath() {
    if (pathPolyline) map.removeLayer(pathPolyline);
    
    let latlngs = waypoints.map(wp => [wp.lat, wp.lng]);
    
    // Dibujar línea desde el robot hasta el primer punto
    if (robotMarker) {
        let robotPos = robotMarker.getLatLng();
        if (robotPos.lat !== 0) latlngs.unshift([robotPos.lat, robotPos.lng]);
    }
    
    pathPolyline = L.polyline(latlngs, { color: '#ff6600', weight: 3, dashArray: '5, 10' }).addTo(map);
}

function clearMap() {
    waypointMarkers.forEach(marker => map.removeLayer(marker));
    waypointMarkers = [];
    waypoints = [];
    if (pathPolyline) { map.removeLayer(pathPolyline); pathPolyline = null; }
}

async function sendMission() {
    if (waypoints.length === 0) { alert("¡Marca puntos en el mapa primero!"); return; }
    try {
        await fetch('/waypoints', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ waypoints: waypoints })
        });
        alert(`¡Misión enviada con ${waypoints.length} puntos! Activa el modo Navegación.`);
    } catch (e) { console.error(e); }
}

// ==========================
// 2. LÓGICA DE CONTROL Y API
// ==========================

async function setRobotMode(mode) {
    try {
        const response = await fetch(`/mode/${mode}`);
        
        if (response.status === 400) {
            alert("⚠️ ¡Error! Enciende el GPS antes de activar la navegación.");
            return;
        }

        if (response.ok && mode !== 'gps') {
            currentMode = mode;
            updateModeButtonUI(mode);
        }
    } catch (error) { console.error(error); }
}

// --- NUEVA FUNCIÓN DE CALIBRACIÓN ---
async function calibrateCompass() {
    // 1. Preguntar confirmación para evitar accidentes
    let confirmar = confirm("⚠️ IMPORTANTE:\n\n¿Has girado el robot físicamente para que apunte al NORTE REAL?\n\nSi no lo has hecho, dale a Cancelar.");
    if (!confirmar) return;

    try {
        const response = await fetch('/calibrate_compass');
        if (response.ok) {
            const data = await response.json();
            alert(`✅ ¡Calibrado!\n\nEl robot ahora sabe que esta posición es el Norte (0°).\nNuevo Offset: ${data.offset}`);
        } else {
            alert("❌ Error al calibrar. Revisa que la brújula esté conectada.");
        }
    } catch (e) {
        console.error(e);
        alert("❌ Error de comunicación con el robot.");
    }
}
// ------------------------------------

function updateModeButtonUI(activeMode) {
    document.querySelectorAll('.mode-btn').forEach(btn => {
        if (btn.dataset.mode !== 'gps') {
            btn.classList.remove('active-mode');
            if (btn.dataset.mode === activeMode) btn.classList.add('active-mode');
        }
    });
    MODE_STATUS_DISPLAY.textContent = activeMode.toUpperCase();
}

function updateGpsButtonUI() {
    if (isGpsActive) {
        GPS_BUTTON.textContent = '📡 GPS ON';
        GPS_BUTTON.classList.add('active-gps');
    } else {
        GPS_BUTTON.textContent = '📡 GPS OFF';
        GPS_BUTTON.classList.remove('active-gps');
    }
}

async function sendMoveCommand(action) {
    // Solo permitimos movimiento manual si estamos en modo manual
    if (currentMode !== 'manual') return; 
    try { await fetch(`/move/${action}`); } catch (e) {}
}

// ==========================
// 3. BUCLE DE DATOS (HEARTBEAT)
// ==========================
async function updateGpsData() {
    try {
        const response = await fetch('/gps_data');
        if (!response.ok) return;
        const data = await response.json();

        // 1. Actualizar Botón GPS (Estado Visual)
        let serverGpsStatus = (data.fix_status !== "Desactivado");
        if (isGpsActive !== serverGpsStatus) {
            isGpsActive = serverGpsStatus;
            updateGpsButtonUI();
        }

        // 2. Actualizar Textos Básicos
        GPS_TXT_LAT.textContent = data.latitude;
        GPS_TXT_LNG.textContent = data.longitude;
        GPS_TXT_SAT.textContent = data.satellites;
        GPS_STATUS_DISPLAY.textContent = data.fix_status;

        // --- LÓGICA DE LA BRÚJULA VISUAL ---
        let heading = data.course || 0;
        
        // A) Texto numérico
        if(COMPASS_TXT) COMPASS_TXT.textContent = Math.round(heading);

        // B) Rotación flecha CSS
        if(COMPASS_ARROW) {
            COMPASS_ARROW.style.transform = `rotate(${heading}deg)`;
        }
        // -----------------------------------

        // 3. Lógica de Navegación y Mapa (Solo si hay GPS Válido)
        if (data.fix_status.includes('Válido') || data.fix_status.includes('FIX')) {
            GPS_STATUS_DISPLAY.style.color = '#007bff';
    
            if (data.lat_raw !== 0.0 && map) {
                let newPos = [data.lat_raw, data.lng_raw];
                
                // Mover marcador
                if(robotMarker) robotMarker.setLatLng(newPos);
                
                // --- ZOOM AUTOMÁTICO (Solo la primera vez) ---
                if (!hasInitialFix) {
                    console.log("📍 Fix inicial detectado. Centrando mapa...");
                    map.invalidateSize(); 
                    map.setView(newPos, 18, { animate: true, pan: { duration: 1 } });
                    hasInitialFix = true;
                }
            }
        }

        // 4. ACTUALIZAR TELEMETRÍA DE NAVEGACIÓN
        if (data.nav_active) {
            if(NAV_PANEL) NAV_PANEL.style.display = 'block';
            
            if(NAV_TXT_WP) NAV_TXT_WP.textContent = '#' + data.nav_wp_index;
            if(NAV_TXT_DIST) NAV_TXT_DIST.textContent = data.nav_dist_m;
            if(NAV_TXT_CURR_HEAD) NAV_TXT_CURR_HEAD.textContent = Math.round(heading);
            if(NAV_TXT_TARG_HEAD) NAV_TXT_TARG_HEAD.textContent = data.nav_target_bearing;
            if(NAV_TXT_ERR) NAV_TXT_ERR.textContent = data.nav_heading_error;

        } else {
            if(NAV_PANEL) NAV_PANEL.style.display = 'none';
        }

    } catch (error) { console.error("Error fetching GPS data:", error); }
}

// ==========================
// 4. EVENT LISTENER (INIT)
// ==========================
window.addEventListener('load', () => {
    
    // Botones de Modo
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.addEventListener('click', () => setRobotMode(btn.dataset.mode));
    });

    // Botones Mapa
    if(BTN_CLEAR_MAP) BTN_CLEAR_MAP.addEventListener('click', clearMap);
    if(BTN_SEND_MISSION) BTN_SEND_MISSION.addEventListener('click', sendMission);

    // Botón Calibrar Brújula (NUEVO)
    if(BTN_CALIBRATE) {
        BTN_CALIBRATE.addEventListener('click', calibrateCompass);
    }

    // Joystick Virtual
    document.querySelectorAll('.move-btn').forEach(btn => {
        const action = btn.dataset.action;
        const stop = () => { if(action !== 'stop') sendMoveCommand('stop'); };
        
        btn.addEventListener('mousedown', (e) => { e.preventDefault(); sendMoveCommand(action); });
        btn.addEventListener('mouseup', stop);
        btn.addEventListener('mouseleave', stop);
        
        btn.addEventListener('touchstart', (e) => { e.preventDefault(); sendMoveCommand(action); });
        btn.addEventListener('touchend', stop);
    });

    // Iniciar bucle de datos
    setInterval(updateGpsData, 1000);
    
    // Iniciar UI
    updateModeButtonUI(currentMode);
    initMap();
});