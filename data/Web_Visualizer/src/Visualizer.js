import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

let camera, scene, renderer, controls, roverMesh, pathLine, plannedPathLine, currentPoints, frontPointCloud, rearPointCloud;
let waypointGroup, detectionGroup;
let markerLayer;
let activeWaypoints = [];
let leftArrow, rightArrow;
let beaconGeo, detectionTex;

let mapCenter = { x: 0, y: 0 };
let cfgRadius = 50;
let cfgTolerance = 10;
let cfgMinScore = 0.0;
let cfgLockGround = true;
let isFetchingMap = false;
let isFollowing = false;
let targetPos = new THREE.Vector3();
let targetHeading = 0.0;
let prevRoverPos = new THREE.Vector3();
const keys = { w: false, a: false, s: false, d: false, q: false, e: false, shift: false };
let lastTime = performance.now();

// ETA Variables
let lastTelemetryTime = 0;
let lastTelemetryPos = new THREE.Vector3();
let avgSpeed = 0.0;
let pathDistance = 0.0;
const speedHistory = [];
let lastPathPoint = null;

// Detection Gallery Tracking
let detectionFilenames = [];
let currentModalIndex = 0;

const typeColors = {};
const typeNames = {};

// State Colors
const stateColors = {
    0: { name: "Idle", color: "#888888" },
    1: { name: "Navigating", color: "#00ffff" },
    2: { name: "Search Pattern", color: "#0000ff" },
    3: { name: "Approach Marker", color: "#ffffff" },
    4: { name: "Approach Object", color: "#ffaa00" },
    5: { name: "Verify Pos", color: "#00550e" },
    6: { name: "Verify Marker", color: "#06ac00" },
    7: { name: "Verify Object", color: "#78ff66" },
    8: { name: "Reversing", color: "#ff0000" },
    9: { name: "Stuck", color: "#330000" }
};

// Detection Colors
const detectColors = {
    10: { name: "Tag (Aruco)", color: "#aa00ff" }, // Purple
    11: { name: "Mallet", color: "#ffa500" },      // Orange
    12: { name: "Bottle", color: "#0088ff" },      // Blue
    13: { name: "Pick", color: "#ffee00" }         // Yellow
};

// Terrain Height Sampler
function getTerrainHeight(rx, rz, defaultY) {
    if (!currentPoints || !currentPoints.geometry || !currentPoints.geometry.attributes.position) return defaultY;
    const positions = currentPoints.geometry.attributes.position.array;
    let sumY = 0;
    let count = 0;
    const radiusSq = 2.25; // 1.5m radius for averaging terrain

    for (let i = 0; i < positions.length; i += 3) {
        const dx = positions[i] - rx;
        const dz = positions[i + 2] - rz;
        if (dx * dx + dz * dz < radiusSq) {
            sumY += positions[i + 1];
            count++;
        }
    }
    return count > 0 ? (sumY / count) : defaultY;
}

// Thick Line / InstancedMesh Renderer (Replaces Firefox-broken LineBasicMaterial)
function createThickPath(vertices, colors, radius, defaultColorHex) {
    if (vertices.length < 6) return null;

    const numSegments = (vertices.length / 3) - 1;
    const cylinderGeo = new THREE.CylinderGeometry(radius, radius, 1, 8, 1, false);
    cylinderGeo.translate(0, 0.5, 0);
    cylinderGeo.rotateX(Math.PI / 2);

    const mat = new THREE.MeshBasicMaterial();
    if (!colors) mat.color.setHex(defaultColorHex);

    const mesh = new THREE.InstancedMesh(cylinderGeo, mat, numSegments);
    const p1 = new THREE.Vector3();
    const p2 = new THREE.Vector3();
    const dummy = new THREE.Object3D();
    const col = new THREE.Color();

    for (let i = 0; i < numSegments; i++) {
        const idx = i * 3;
        p1.set(vertices[idx], vertices[idx + 1], vertices[idx + 2]);
        p2.set(vertices[idx + 3], vertices[idx + 4], vertices[idx + 5]);

        const dist = p1.distanceTo(p2);
        if (dist < 0.001) {
            dummy.scale.set(0, 0, 0);
            dummy.updateMatrix();
            mesh.setMatrixAt(i, dummy.matrix);
            continue;
        }

        dummy.position.copy(p1);
        dummy.lookAt(p2);
        dummy.scale.set(1, 1, dist);
        dummy.updateMatrix();
        mesh.setMatrixAt(i, dummy.matrix);

        if (colors) {
            col.setRGB(colors[idx], colors[idx + 1], colors[idx + 2]);
            mesh.setColorAt(i, col);
        }
    }

    mesh.instanceMatrix.needsUpdate = true;
    if (colors) mesh.instanceColor.needsUpdate = true;
    return mesh;
}

init();
animate();

function init() {
    markerLayer = document.getElementById('marker-layer');
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x111111);
    scene.add(new THREE.GridHelper(100, 100));
    scene.add(new THREE.AxesHelper(2));

    camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 10000);
    camera.position.set(0, 10, -10);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    document.body.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.maxDistance = 5000;

    const geometry = new THREE.BoxGeometry(1, 0.5, 1.5);
    geometry.translate(0, 0.25, 0);
    const material = new THREE.MeshBasicMaterial({ color: 0xff00ff, wireframe: true });
    roverMesh = new THREE.Mesh(geometry, material);
    scene.add(roverMesh);

    // Drive Vectors (Arrows)
    const arrowDir = new THREE.Vector3(0, 0, -1);
    const arrowOrigin = new THREE.Vector3(0, 0, 0);
    const arrowLen = 1;
    const arrowCol = 0xffff00;
    leftArrow = new THREE.ArrowHelper(arrowDir, arrowOrigin, arrowLen, arrowCol);
    rightArrow = new THREE.ArrowHelper(arrowDir, arrowOrigin, arrowLen, arrowCol);
    roverMesh.add(leftArrow);
    roverMesh.add(rightArrow);
    leftArrow.position.set(-0.6, 0, 0);
    rightArrow.position.set(0.6, 0, 0);

    beaconGeo = new THREE.BoxGeometry(0.5, 10000, 0.5);
    const canvas = document.createElement('canvas');
    canvas.width = 32; canvas.height = 32;
    const ctx = canvas.getContext('2d');
    ctx.beginPath();
    ctx.arc(16, 16, 14, 0, 2 * Math.PI);
    ctx.fillStyle = 'white';
    ctx.fill();
    detectionTex = new THREE.CanvasTexture(canvas);

    waypointGroup = new THREE.Group();
    scene.add(waypointGroup);

    detectionGroup = new THREE.Group();
    scene.add(detectionGroup);

    const config = [
        { id: 0, name: "NAV", color: "#00ffff" },
        { id: 1, name: "TAG", color: "#ffffff" },
        { id: 2, name: "MALLET", color: "#ffa500" },
        { id: 3, name: "BOTTLE", color: "#0088ff" },
        { id: 4, name: "PICK", color: "#ffee00" },
        { id: 5, name: "OBJ", color: "#aaaaaa" },
        { id: 6, name: "OBSTACLE", color: "#ff0000" },
        { id: 7, name: "UNKNOWN", color: "#000000" },
        { id: 8, name: "GOAL REACHED", color: "#00ff00" }
    ];

    config.forEach(c => {
        typeNames[c.id] = c.name;
        typeColors[c.id] = new THREE.Color(c.color);
    });

    // Legend: Detections
    const detLegendDiv = document.getElementById('det-legend');
    for (const [id, data] of Object.entries(detectColors)) {
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `<div class="circle-box" style="background:${data.color}"></div><span>${data.name}</span>`;
        detLegendDiv.appendChild(item);
    }

    // Legend: States
    const stateLegendDiv = document.getElementById('state-legend');
    for (const [id, data] of Object.entries(stateColors)) {
        const item = document.createElement('div');
        item.className = 'legend-item';
        item.innerHTML = `<div class="color-box" style="background:${data.color}"></div><span>${data.name}</span>`;
        stateLegendDiv.appendChild(item);
    }

    // Toggles & Inputs
    const cbGround = document.getElementById('cb-ground');
    if (cbGround) {
        cbGround.addEventListener('change', (e) => {
            cfgLockGround = e.target.checked;
            if (cfgLockGround) {
                targetPos.y = getTerrainHeight(targetPos.x, targetPos.z, targetPos.y);
            }
        });
    }
    document.getElementById('sl-rad').oninput = (e) => {
        cfgRadius = parseInt(e.target.value);
        document.getElementById('val-rad').innerText = cfgRadius;
        checkBoundary(true);
    };
    document.getElementById('sl-tol').oninput = (e) => {
        cfgTolerance = parseInt(e.target.value);
        document.getElementById('val-tol').innerText = cfgTolerance;
    };
    document.getElementById('sl-score').oninput = (e) => {
        cfgMinScore = parseFloat(e.target.value);
        document.getElementById('val-score').innerText = cfgMinScore.toFixed(2);
        checkBoundary(true);
    };

    // UI Layer Collapsible Toggle
    const settingsToggle = document.getElementById('settings-toggle');
    const settingsContent = document.getElementById('settings-content');
    settingsToggle.addEventListener('click', () => {
        if (settingsContent.style.display === 'none') {
            settingsContent.style.display = 'block';
            settingsToggle.innerHTML = 'Settings &#9660;';
        } else {
            settingsContent.style.display = 'none';
            settingsToggle.innerHTML = 'Settings &#9654;';
        }
    });

    window.addEventListener('keydown', (e) => onKey(e, true));
    window.addEventListener('keyup', (e) => onKey(e, false));
    window.addEventListener('resize', onWindowResize);

    window.toggleFollow = () => {
        isFollowing = !isFollowing;
        document.getElementById('follow-btn').classList.toggle('active', isFollowing);
        document.getElementById('status').innerText = isFollowing ? "Status: Locked" : "Status: Free Cam";
        if (isFollowing) controls.target.copy(roverMesh.position);
    };
    window.snapToRover = () => {
        camera.position.copy(roverMesh.position).add(new THREE.Vector3(0, 10, -10));
        controls.target.copy(roverMesh.position);
    };

    requestTelemetryLoop();
    requestPointCloudLoop();
    setInterval(fetchPlannedPath, 2000);
    setInterval(fetchWaypoints, 2000);
    setInterval(fetchDetections, 1000);
    setInterval(fetchDetectionsList, 5000);
    fetchMapSquare(0, 0);
}

// --- RECURSIVE LOOP ---
async function requestTelemetryLoop() {
    if (!document.hidden) await fetchTelemetry();
    setTimeout(requestTelemetryLoop, 50);
}

async function fetchTelemetry() {
    try {
        const response = await fetch('/api/telemetry');
        if (!response.ok) return;
        const buffer = await response.arrayBuffer();
        updateTelemetry(buffer);
    } catch (e) { }
}

async function fetchPlannedPath() {
    try {
        const response = await fetch('/api/planned_path');
        const buffer = await response.arrayBuffer();
        updatePlannedPath(buffer);
    } catch (e) { }
}

async function fetchWaypoints() {
    try {
        const response = await fetch('/api/waypoints');
        const buffer = await response.arrayBuffer();
        updateWaypoints(buffer);
    } catch (e) { }
}

async function fetchDetections() {
    try {
        const response = await fetch('/api/detections');
        const buffer = await response.arrayBuffer();
        updateDetections(buffer);
    } catch (e) { }
}

async function fetchDetectionsList() {
    try {
        const response = await fetch('/api/detection_list');
        const filenames = await response.json();
        updateDetectionGallery(filenames);
    } catch (e) {
        console.error('Failed to fetch detection list:', e);
    }
}

function updateDetectionGallery(filenames) {
    detectionFilenames = filenames;
    const panel = document.getElementById('detection-panel');
    const gallery = document.getElementById('detection-gallery-items');
    const header = panel ? panel.querySelector('h3') : null;
    if (!gallery) return;

    gallery.innerHTML = '';

    if (filenames.length === 0) {
        if (panel) panel.style.width = 'auto';
        if (header) header.style.display = 'none';
        gallery.innerHTML = '<div style="color:#aaa; font-size:12px; text-align:center;">No detections</div>';
        return;
    }

    if (panel) panel.style.width = '250px';
    if (header) header.style.display = 'block';

    const latestIndex = filenames.length - 1;
    const filename = filenames[latestIndex];

    const item = document.createElement('div');
    item.className = 'gallery-item';

    const img = document.createElement('img');
    img.src = `/detections/${filename}`;
    img.alt = filename;
    img.title = "Click to view full gallery";
    img.addEventListener('click', () => showDetectionModal(latestIndex));

    const info = document.createElement('div');
    info.style.color = '#fff';
    info.style.fontSize = '14px';
    info.style.textAlign = 'center';
    info.style.marginTop = '8px';
    info.innerText = `View all ${filenames.length} images`;

    item.appendChild(img);
    item.appendChild(info);
    gallery.appendChild(item);
}

window.showDetectionModal = function (index) {
    const modal = document.getElementById('detection-modal');
    const img = document.getElementById('modal-image');
    const caption = document.getElementById('modal-caption');
    const openBtn = document.getElementById('modal-open-btn');

    if (modal && img && caption && detectionFilenames.length > 0) {
        currentModalIndex = index;
        const filename = detectionFilenames[currentModalIndex];
        const src = `/detections/${filename}`;

        img.src = src;
        caption.innerText = `${filename} (${currentModalIndex + 1} of ${detectionFilenames.length})`;

        if (openBtn) {
            openBtn.onclick = () => window.open(src, '_blank');
        }
        modal.style.display = 'flex';
    }
}

window.closeDetectionModal = function () {
    const modal = document.getElementById('detection-modal');
    if (modal) modal.style.display = 'none';
}

window.nextDetection = function (e) {
    e.stopPropagation();
    if (detectionFilenames.length === 0) return;
    let newIdx = currentModalIndex + 1;
    if (newIdx >= detectionFilenames.length) newIdx = 0;
    showDetectionModal(newIdx);
}

window.prevDetection = function (e) {
    e.stopPropagation();
    if (detectionFilenames.length === 0) return;
    let newIdx = currentModalIndex - 1;
    if (newIdx < 0) newIdx = detectionFilenames.length - 1;
    showDetectionModal(newIdx);
}

function updateArrow(arrow, power) {
    const absPwr = Math.abs(power);
    const dir = power >= 0 ? new THREE.Vector3(0, 0, -1) : new THREE.Vector3(0, 0, 1);
    arrow.setDirection(dir);
    arrow.setLength(Math.max(absPwr * 2.0, 0.001), 0.2, 0.1);
    const col = power >= 0 ? 0x00ff00 : 0xff0000;
    arrow.setColor(col);
}

function updateTelemetry(buffer) {
    const view = new DataView(buffer);
    const rx = view.getFloat32(0, true);
    const ry = view.getFloat32(4, true);
    const rz = view.getFloat32(8, true);
    const rh = view.getFloat32(12, true);

    // Calculate Speed using actual un-flattened 3D position
    const now = performance.now();
    const newPos = new THREE.Vector3(rx, ry, -rz);
    if (lastTelemetryTime > 0) {
        const dt = (now - lastTelemetryTime) / 1000.0;
        if (dt > 0.1) {
            const dist = newPos.distanceTo(lastTelemetryPos);
            const instSpeed = dist / dt;
            speedHistory.push(instSpeed);
            if (speedHistory.length > 20) speedHistory.shift();
            avgSpeed = speedHistory.reduce((a, b) => a + b, 0) / speedHistory.length;
        }
    }
    lastTelemetryPos.copy(newPos);
    lastTelemetryTime = now;

    // Drive Powers
    const leftPwr = view.getFloat32(16, true);
    const rightPwr = view.getFloat32(20, true);
    updateArrow(leftArrow, leftPwr);
    updateArrow(rightArrow, rightPwr);

    let targetY = ry;
    if (cfgLockGround) {
        targetY = getTerrainHeight(rx, -rz, ry);
    }
    targetPos.set(rx, targetY, -rz);
    targetHeading = -rh * (Math.PI / 180.0);

    checkBoundary(false);

    const pathCount = view.getUint32(24, true); // Offset 24
    if (pathCount > 0) {
        if (pathLine) {
            scene.remove(pathLine);
            if (pathLine.geometry) pathLine.geometry.dispose();
            if (pathLine.material) pathLine.material.dispose();
        }
        const floats = new Float32Array(buffer, 28, pathCount * 4); // Offset 28
        const vertices = [];
        const colors = [];
        const c = new THREE.Color();

        for (let i = 0; i < floats.length; i += 4) {
            vertices.push(floats[i], floats[i + 1], -floats[i + 2]);
            const state = Math.floor(floats[i + 3]);
            const hex = stateColors[state] ? stateColors[state].color : "#ffffff";
            c.set(hex);
            colors.push(c.r, c.g, c.b);
        }

        pathLine = createThickPath(vertices, colors, 0.1, 0xffffff);
        if (pathLine) scene.add(pathLine);
    }
}

function updatePlannedPath(buffer) {
    const view = new DataView(buffer);
    const count = view.getUint32(0, true);
    if (plannedPathLine) {
        scene.remove(plannedPathLine);
        if (plannedPathLine.geometry) plannedPathLine.geometry.dispose();
        if (plannedPathLine.material) plannedPathLine.material.dispose();
        plannedPathLine = null;
    }

    pathDistance = 0.0;
    lastPathPoint = null;

    if (count > 0) {
        const floats = new Float32Array(buffer, 4, count * 3);
        const vertices = [];
        for (let i = 0; i < floats.length; i += 3) {
            vertices.push(floats[i], floats[i + 1], -floats[i + 2]);
        }

        // Calculate total path distance (sum of segments)
        // Add distance from rover to first point
        if (vertices.length >= 3) {
            const firstPt = new THREE.Vector3(vertices[0], vertices[1], vertices[2]);
            pathDistance += targetPos.distanceTo(firstPt);
            // Store Last Point
            const lastIdx = vertices.length - 3;
            lastPathPoint = new THREE.Vector3(vertices[lastIdx], vertices[lastIdx + 1], vertices[lastIdx + 2]);
        }
        // Add segments
        for (let i = 0; i < vertices.length - 3; i += 3) {
            const p1 = new THREE.Vector3(vertices[i], vertices[i + 1], vertices[i + 2]);
            const p2 = new THREE.Vector3(vertices[i + 3], vertices[i + 4], vertices[i + 5]);
            pathDistance += p1.distanceTo(p2);
        }

        plannedPathLine = createThickPath(vertices, null, 0.1, 0xeeff00);
        if (plannedPathLine) scene.add(plannedPathLine);
    }
}

function updateWaypoints(buffer) {
    while (waypointGroup.children.length > 0) {
        const child = waypointGroup.children[0];
        waypointGroup.remove(child);
        if (child.material) child.material.dispose();
    }
    markerLayer.innerHTML = '';
    activeWaypoints = [];

    const view = new DataView(buffer);
    const count = view.getUint32(0, true);
    if (count === 0) return;

    let offset = 4;

    for (let i = 0; i < count; i++) {
        const x = view.getFloat32(offset, true);
        const y = view.getFloat32(offset + 4, true);
        const z = view.getFloat32(offset + 8, true);
        const type = view.getInt32(offset + 12, true);
        offset += 16;

        const col = typeColors[type] || typeColors[7];
        const beaconMat = new THREE.MeshBasicMaterial({
            color: col,
            transparent: true,
            opacity: 0.3,
            depthTest: false
        });
        const beacon = new THREE.Mesh(beaconGeo, beaconMat);
        beacon.position.set(x, 0, -z);
        waypointGroup.add(beacon);

        const div = document.createElement('div');
        div.className = 'hud-marker';
        div.innerText = typeNames[type] || "UNK";
        div.style.borderColor = "#" + col.getHexString();
        markerLayer.appendChild(div);

        activeWaypoints.push({
            div: div,
            pos: new THREE.Vector3(x, 0, -z)
        });
    }
}

function updateDetections(buffer) {
    while (detectionGroup.children.length > 0) {
        const child = detectionGroup.children[0];
        detectionGroup.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
    }

    const view = new DataView(buffer);
    const count = view.getUint32(0, true);
    if (count === 0) return;

    let offset = 4;

    for (let i = 0; i < count; i++) {
        const x = view.getFloat32(offset, true);
        const y = view.getFloat32(offset + 4, true);
        const z = view.getFloat32(offset + 8, true);
        const type = view.getInt32(offset + 12, true);
        offset += 16;

        let col = "#ffffff";
        if (detectColors[type]) col = detectColors[type].color;

        const mat = new THREE.PointsMaterial({
            color: col,
            map: detectionTex,
            size: 2.0, // Large persistent dot
            sizeAttenuation: true,
            alphaTest: 0.5,
            transparent: true
        });
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute([x, y, -z], 3));

        const pt = new THREE.Points(geo, mat);
        detectionGroup.add(pt);
    }
}

function updateHUD() {
    const width = window.innerWidth;
    const height = window.innerHeight;
    const pad = 30;

    // Update ETA Box
    const etaBox = document.getElementById('eta-box');

    // Reached End Logic
    let bReached = false;
    if (lastPathPoint && roverMesh.position.distanceTo(lastPathPoint) < 2.0) {
        bReached = true;
    }

    if (bReached) {
        etaBox.innerText = "Status: Reached End of Path";
        etaBox.style.color = "#00ff00"; // Green
    } else {
        etaBox.style.color = "#0f0"; // Default Green
        if (avgSpeed < 0.05) {
            etaBox.innerText = "ETA: Stopped";
        } else {
            const timeSec = pathDistance / avgSpeed;
            if (!isFinite(timeSec) || timeSec < 0) {
                etaBox.innerText = "ETA: --:--";
            } else {
                const min = Math.floor(timeSec / 60);
                const sec = Math.floor(timeSec % 60);
                etaBox.innerText = `ETA: ${min}m ${sec}s (${avgSpeed.toFixed(2)} m/s)`;
            }
        }
    }

    activeWaypoints.forEach(wp => {
        const target = wp.pos.clone();
        target.y = roverMesh.position.y + 3.0;
        target.project(camera);

        let x = (target.x * .5 + .5) * width;
        let y = (target.y * -.5 + .5) * height;

        const isBehind = target.z > 1;

        if (isBehind) {
            x = width - x;
            y = height - y;
        }

        const cx = width / 2;
        const cy = height / 2;
        const dx = x - cx;
        const dy = y - cy;

        if (!isBehind && x >= pad && x <= width - pad && y >= pad && y <= height - pad) {
            y -= 40;
            wp.div.style.opacity = "0.9";
        } else {
            let t = Infinity;
            if (dx > 0) t = Math.min(t, (width - pad - cx) / dx);
            if (dx < 0) t = Math.min(t, (pad - cx) / dx);
            if (dy > 0) t = Math.min(t, (height - pad - cy) / dy);
            if (dy < 0) t = Math.min(t, (pad - cy) / dy);

            x = cx + dx * t;
            y = cy + dy * t;
            wp.div.style.opacity = "0.6";
        }

        wp.div.style.left = x + 'px';
        wp.div.style.top = y + 'px';

        // Use 2D horizontal distance for distance display.
        const dxPos = roverMesh.position.x - wp.pos.x;
        const dzPos = roverMesh.position.z - wp.pos.z;
        const dist = Math.sqrt(dxPos * dxPos + dzPos * dzPos);

        wp.div.innerText = `${wp.div.innerText.split(' ')[0]} ${Math.round(dist)}m`;
    });
}

function checkBoundary(force) {
    if (isFetchingMap && !force) return;
    const roverX = targetPos.x;
    const roverN = -targetPos.z;
    const distE = Math.abs(roverX - mapCenter.x);
    const distN = Math.abs(roverN - mapCenter.y);
    const limit = cfgRadius - cfgTolerance;

    if (force || distE > limit || distN > limit) {
        fetchMapSquare(roverX, roverN);
    }
}

async function fetchMapSquare(x, y) {
    if (isFetchingMap) return;
    isFetchingMap = true;
    try {
        const url = `/api/map?x=${x.toFixed(2)}&y=${y.toFixed(2)}&r=${cfgRadius}&s=${cfgMinScore}`;
        const response = await fetch(url);
        const buffer = await response.arrayBuffer();
        loadMapPoints(buffer);
        mapCenter = { x: x, y: y };
    } catch (e) { console.error(e); }
    isFetchingMap = false;
}

function loadMapPoints(buffer) {
    const view = new DataView(buffer);
    const count = view.getUint32(0, true);

    if (currentPoints) {
        scene.remove(currentPoints);
        currentPoints.geometry.dispose();
        currentPoints.material.dispose();
        currentPoints = null;
    }

    if (count === 0) {
        document.getElementById('stats').innerText = "Points: 0";
        return;
    }

    const pts = [];
    const colors = [];
    const c = new THREE.Color();
    const floats = new Float32Array(buffer, 4, count * 4);

    for (let i = 0; i < floats.length; i += 4) {
        pts.push(floats[i], floats[i + 1], -floats[i + 2]);
        c.setHSL(floats[i + 3] * 0.33, 1.0, 0.5);
        colors.push(c.r, c.g, c.b);
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
    geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    const mat = new THREE.PointsMaterial({ size: 0.5, vertexColors: true });

    currentPoints = new THREE.Points(geo, mat);
    scene.add(currentPoints);
    document.getElementById('stats').innerText = "Points: " + count;
}

// --- RECURSIVE LOOP ---
async function requestPointCloudLoop() {
    if (!document.hidden) await fetchPointCloud();
    setTimeout(requestPointCloudLoop, 500);
}

async function fetchPointCloud() {
    try {
        const url = `/api/point_cloud?density=6`;
        const response = await fetch(url);
        const buffer = await response.arrayBuffer();
        loadPointCloud(buffer);
    } catch (e) { console.error(e); }
}


function loadPointCloud(buffer) {

    if (!roverMesh) return;
    if (!frontPointCloud) {
        frontPointCloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ size: 0.1, color: 0x00ffff }));
        frontPointCloud.geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(0), 3));
        frontPointCloud.geometry.attributes.position.usage = THREE.DynamicDrawUsage;
        frontPointCloud.position.set(0, 1, 0);
        roverMesh.add(frontPointCloud);
        console.log("Created front point cloud.");
    }
    if (!rearPointCloud) {
        rearPointCloud = new THREE.Points(new THREE.BufferGeometry(), new THREE.PointsMaterial({ size: 0.1, color: 0xff0000 }));
        rearPointCloud.geometry.setAttribute('position', new THREE.Float32BufferAttribute(new Float32Array(0), 3));
        rearPointCloud.geometry.attributes.position.usage = THREE.DynamicDrawUsage;
        rearPointCloud.position.set(0, 1, 0);
        rearPointCloud.rotation.set(0, Math.PI, 0);
        roverMesh.add(rearPointCloud);
        console.log("Created rear point cloud.");
    }

    const view = new DataView(buffer);

    // Front point cloud
    const frontCount = view.getUint32(0, true) * 3;
    let frontPoints = frontPointCloud.geometry.attributes.position.array;
    if (frontPoints.length < frontCount) {
        frontPoints = new Float32Array(frontCount);
        frontPointCloud.geometry.setAttribute('position', new THREE.Float32BufferAttribute(frontPoints, 3));
        console.log("Resized front point cloud to", frontCount, "floats.");
    }
    const frontView = new DataView(buffer, 4, frontCount * 4);
    for (let i = 0; i < frontCount; i += 3) {
        frontPoints[i] = frontView.getFloat32(i * 4, true);
        frontPoints[i + 1] = frontView.getFloat32((i * 4) + 4, true);
        frontPoints[i + 2] = -frontView.getFloat32((i * 4) + 8, true);
    }
    frontPointCloud.geometry.setDrawRange(0, frontCount / 3);
    frontPointCloud.geometry.attributes.position.needsUpdate = true;

    // Back point cloud
    const rearCount = view.getUint32(4 + frontCount * 4, true) * 3;
    let rearPoints = rearPointCloud.geometry.attributes.position.array;
    if (rearPoints.length < rearCount) {
        rearPoints = new Float32Array(rearCount);
        rearPointCloud.geometry.setAttribute('position', new THREE.Float32BufferAttribute(rearPoints, 3));
        console.log("Resized rear point cloud to", rearCount, "floats.");
    }
    const rearView = new DataView(buffer, 4 + frontCount * 4 + 4, rearCount * 4);
    for (let i = 0; i < rearCount; i += 3) {
        rearPoints[i] = rearView.getFloat32(i * 4, true);
        rearPoints[i + 1] = rearView.getFloat32((i * 4) + 4, true);
        rearPoints[i + 2] = -rearView.getFloat32((i * 4) + 8, true);
    }
    rearPointCloud.geometry.setDrawRange(0, rearCount / 3);
    rearPointCloud.geometry.attributes.position.needsUpdate = true;
}

function onKey(e, p) {
    if (keys.hasOwnProperty(e.key.toLowerCase())) keys[e.key.toLowerCase()] = p;
    if (e.key === 'Shift') keys.shift = p;
    if (p && e.key === 'f') window.toggleFollow();
    if (p && e.key === ' ') window.snapToRover();
}
function onWindowResize() { camera.aspect = window.innerWidth / window.innerHeight; camera.updateProjectionMatrix(); renderer.setSize(window.innerWidth, window.innerHeight); }

function animate() {
    requestAnimationFrame(animate);
    const now = performance.now();
    const dt = Math.min((now - lastTime) / 1000.0, 0.1);
    lastTime = now;

    prevRoverPos.copy(roverMesh.position);
    const lerpFactor = 5.0 * dt;
    roverMesh.position.lerp(targetPos, lerpFactor);
    const dRot = targetHeading - roverMesh.rotation.y;
    roverMesh.rotation.y += Math.atan2(Math.sin(dRot), Math.cos(dRot)) * lerpFactor;

    if (isFollowing) {
        camera.position.add(new THREE.Vector3().subVectors(roverMesh.position, prevRoverPos));
        controls.target.copy(roverMesh.position);
    } else {
        const spd = (keys.shift ? 15 : 5) * dt;
        const fwd = new THREE.Vector3(); camera.getWorldDirection(fwd); fwd.y = 0; fwd.normalize();
        const rgt = new THREE.Vector3().crossVectors(fwd, camera.up).normalize();
        if (keys.w) camera.position.addScaledVector(fwd, spd);
        if (keys.s) camera.position.addScaledVector(fwd, -spd);
        if (keys.d) camera.position.addScaledVector(rgt, spd);
        if (keys.a) camera.position.addScaledVector(rgt, -spd);
        if (keys.q) camera.position.y += spd;
        if (keys.e) camera.position.y -= spd;
        controls.target.add(new THREE.Vector3(0, 0, 0).addScaledVector(fwd, (keys.w - keys.s) * spd).addScaledVector(rgt, (keys.d - keys.a) * spd));
    }
    controls.update();

    updateHUD();

    renderer.render(scene, camera);
}