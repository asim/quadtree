// QuadTree UI JavaScript
// Depends on config object being set before this script loads

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

let viewX = 0;
let viewY = 0;
let zoom = 1;
let points = [];
let isDragging = false;
let lastMouseX = 0;
let lastMouseY = 0;

function toggleSidebar() {
    const sidebar = document.getElementById('sidebar');
    const overlay = document.getElementById('sidebarOverlay');
    const isOpen = sidebar.classList.toggle('open');
    if (isOpen) {
        overlay.classList.add('visible');
    } else {
        overlay.classList.remove('visible');
    }
}

function zoomIn() {
    zoom *= 1.3;
    zoom = Math.min(zoom, 100);
    draw();
}

function zoomOut() {
    zoom *= 0.7;
    zoom = Math.max(zoom, 0.1);
    draw();
}

function resetView() {
    viewX = 0;
    viewY = 0;
    zoom = 1;
    draw();
}

function isMobile() {
    return window.innerWidth <= 768;
}

function resizeCanvas() {
    const container = canvas.parentElement;
    canvas.width = container.clientWidth;
    canvas.height = container.clientHeight;
    draw();
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Convert screen coordinates to world coordinates
function screenToWorld(screenX, screenY) {
    const worldX = (screenX - canvas.width / 2) / zoom + viewX;
    const worldY = (canvas.height / 2 - screenY) / zoom + viewY;
    return { x: worldX, y: worldY };
}

// Convert world coordinates to screen coordinates
function worldToScreen(worldX, worldY) {
    const screenX = (worldX - viewX) * zoom + canvas.width / 2;
    const screenY = canvas.height / 2 - (worldY - viewY) * zoom;
    return { x: screenX, y: screenY };
}

function draw() {
    ctx.fillStyle = '#ffffff';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    
    // Draw grid
    drawGrid();
    
    // Draw axes
    ctx.strokeStyle = '#cbd5e0';
    ctx.lineWidth = 2;
    const origin = worldToScreen(0, 0);
    ctx.beginPath();
    ctx.moveTo(origin.x, 0);
    ctx.lineTo(origin.x, canvas.height);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, origin.y);
    ctx.lineTo(canvas.width, origin.y);
    ctx.stroke();
    
    // Draw points
    points.forEach(point => {
        const screen = worldToScreen(point.x, point.y);
        
        // Only draw if on screen
        if (screen.x >= -20 && screen.x <= canvas.width + 20 &&
            screen.y >= -20 && screen.y <= canvas.height + 20) {
            
            // Draw radius circle if present
            if (point.radius && point.radius > 0) {
                const radiusPixels = point.radius * zoom;
                ctx.strokeStyle = 'rgba(66, 153, 225, 0.3)';
                ctx.fillStyle = 'rgba(66, 153, 225, 0.1)';
                ctx.lineWidth = 1;
                ctx.beginPath();
                ctx.arc(screen.x, screen.y, radiusPixels, 0, 2 * Math.PI);
                ctx.fill();
                ctx.stroke();
            }
            
            // Draw point
            ctx.fillStyle = '#4299e1';
            ctx.beginPath();
            ctx.arc(screen.x, screen.y, 6, 0, 2 * Math.PI);
            ctx.fill();
            
            // Draw label
            const label = point.name || point.data || '';
            if (label) {
                ctx.fillStyle = '#2d3748';
                ctx.font = '12px -apple-system, BlinkMacSystemFont, sans-serif';
                ctx.fillText(label, screen.x + 10, screen.y - 10);
            }
        }
    });
    
    // Update info
    document.getElementById('currentX').textContent = viewX.toFixed(2);
    document.getElementById('currentY').textContent = viewY.toFixed(2);
    document.getElementById('currentZoom').textContent = zoom.toFixed(2);
}

function drawGrid() {
    // Calculate grid spacing based on zoom
    let gridSize = 50;
    if (zoom < 0.5) gridSize = 200;
    else if (zoom < 1) gridSize = 100;
    else if (zoom > 2) gridSize = 25;
    else if (zoom > 5) gridSize = 10;
    
    ctx.strokeStyle = '#f0f0f0';
    ctx.lineWidth = 1;
    
    // Calculate visible range
    const topLeft = screenToWorld(0, 0);
    const bottomRight = screenToWorld(canvas.width, canvas.height);
    
    // Draw vertical lines
    const startX = Math.floor(topLeft.x / gridSize) * gridSize;
    for (let x = startX; x <= bottomRight.x; x += gridSize) {
        const screen = worldToScreen(x, 0);
        ctx.beginPath();
        ctx.moveTo(screen.x, 0);
        ctx.lineTo(screen.x, canvas.height);
        ctx.stroke();
    }
    
    // Draw horizontal lines
    const startY = Math.floor(bottomRight.y / gridSize) * gridSize;
    for (let y = startY; y <= topLeft.y; y += gridSize) {
        const screen = worldToScreen(0, y);
        ctx.beginPath();
        ctx.moveTo(0, screen.y);
        ctx.lineTo(canvas.width, screen.y);
        ctx.stroke();
    }
}

// Mouse controls
canvas.addEventListener('mousedown', (e) => {
    isDragging = true;
    lastMouseX = e.clientX;
    lastMouseY = e.clientY;
});

canvas.addEventListener('mousemove', (e) => {
    if (isDragging) {
        const dx = e.clientX - lastMouseX;
        const dy = e.clientY - lastMouseY;
        viewX -= dx / zoom;
        viewY += dy / zoom;
        lastMouseX = e.clientX;
        lastMouseY = e.clientY;
        draw();
    }
});

canvas.addEventListener('mouseup', () => {
    isDragging = false;
});

canvas.addEventListener('mouseleave', () => {
    isDragging = false;
});

canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
    zoom *= zoomFactor;
    zoom = Math.max(0.1, Math.min(zoom, 100));
    draw();
});

// Touch controls
let touchStartDistance = 0;
let touchStartZoom = 1;

canvas.addEventListener('touchstart', (e) => {
    e.preventDefault();
    if (e.touches.length === 1) {
        isDragging = true;
        lastMouseX = e.touches[0].clientX;
        lastMouseY = e.touches[0].clientY;
    } else if (e.touches.length === 2) {
        isDragging = false;
        const dx = e.touches[0].clientX - e.touches[1].clientX;
        const dy = e.touches[0].clientY - e.touches[1].clientY;
        touchStartDistance = Math.sqrt(dx * dx + dy * dy);
        touchStartZoom = zoom;
    }
});

canvas.addEventListener('touchmove', (e) => {
    e.preventDefault();
    if (e.touches.length === 1 && isDragging) {
        const dx = e.touches[0].clientX - lastMouseX;
        const dy = e.touches[0].clientY - lastMouseY;
        viewX -= dx / zoom;
        viewY += dy / zoom;
        lastMouseX = e.touches[0].clientX;
        lastMouseY = e.touches[0].clientY;
        draw();
    } else if (e.touches.length === 2) {
        const dx = e.touches[0].clientX - e.touches[1].clientX;
        const dy = e.touches[0].clientY - e.touches[1].clientY;
        const distance = Math.sqrt(dx * dx + dy * dy);
        if (touchStartDistance > 0) {
            zoom = touchStartZoom * (distance / touchStartDistance);
            zoom = Math.max(0.1, Math.min(zoom, 100));
            draw();
        }
    }
});

canvas.addEventListener('touchend', (e) => {
    e.preventDefault();
    if (e.touches.length === 0) {
        isDragging = false;
    } else if (e.touches.length === 1) {
        lastMouseX = e.touches[0].clientX;
        lastMouseY = e.touches[0].clientY;
    }
});

// Keyboard controls
document.addEventListener('keydown', (e) => {
    // Don't handle if focused on input
    if (e.target.tagName === 'INPUT') return;
    
    const step = 50 / zoom;
    switch(e.key) {
        case 'ArrowUp':
            viewY += step;
            draw();
            e.preventDefault();
            break;
        case 'ArrowDown':
            viewY -= step;
            draw();
            e.preventDefault();
            break;
        case 'ArrowLeft':
            viewX -= step;
            draw();
            e.preventDefault();
            break;
        case 'ArrowRight':
            viewX += step;
            draw();
            e.preventDefault();
            break;
        case '+':
        case '=':
            zoomIn();
            e.preventDefault();
            break;
        case '-':
            zoomOut();
            e.preventDefault();
            break;
    }
});

function setStatus(message, duration = 3000) {
    const status = document.getElementById('status');
    status.textContent = message;
    status.classList.add('visible');
    if (duration > 0) {
        setTimeout(() => {
            status.classList.remove('visible');
        }, duration);
    }
}

// API URL builder
function apiUrl(endpoint) {
    return config.apiBase + endpoint;
}

async function addPoint() {
    const xValue = document.getElementById('addX').value;
    const yValue = document.getElementById('addY').value;
    const data = document.getElementById('addData').value;
    await addPointWithValues(xValue, yValue, data);
}

async function addPointSidebar() {
    const xValue = document.getElementById('addXSidebar').value;
    const yValue = document.getElementById('addYSidebar').value;
    const data = document.getElementById('addDataSidebar').value;
    await addPointWithValues(xValue, yValue, data);
    if (isMobile()) toggleSidebar();
}

async function addPointWithValues(xValue, yValue, data) {
    if (xValue === '' || yValue === '') {
        setStatus('❌ Please enter X and Y coordinates');
        return;
    }
    
    const x = parseFloat(xValue);
    const y = parseFloat(yValue);
    
    if (isNaN(x) || isNaN(y)) {
        setStatus('❌ Invalid coordinates - must be numbers');
        return;
    }
    
    try {
        const response = await fetch(apiUrl(config.pointsEndpoint), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ x, y, data: data || null })
        });
        
        if (response.ok) {
            setStatus('✓ ' + config.pointLabel + ' added');
            loadAllPoints();
        } else {
            setStatus('❌ Failed to add ' + config.pointLabel.toLowerCase());
        }
    } catch (error) {
        setStatus('❌ Error: ' + error.message);
    }
}

async function deletePoint(id) {
    if (config.readOnly) return;
    
    try {
        const response = await fetch(apiUrl(config.pointsEndpoint + '/' + id), {
            method: 'DELETE'
        });
        
        if (response.ok) {
            setStatus('✓ ' + config.pointLabel + ' deleted');
            loadAllPoints();
        } else {
            setStatus('❌ Failed to delete ' + config.pointLabel.toLowerCase());
        }
    } catch (error) {
        setStatus('❌ Error: ' + error.message);
    }
}

async function search() {
    const radiusValue = document.getElementById('searchRadius').value;
    await searchWithRadius(radiusValue);
}

async function searchSidebar() {
    const radiusValue = document.getElementById('searchRadiusSidebar').value;
    await searchWithRadius(radiusValue);
    if (isMobile()) toggleSidebar();
}

async function searchWithRadius(radiusValue) {
    const radius = parseFloat(radiusValue);
    
    if (radiusValue === '' || isNaN(radius) || radius <= 0) {
        setStatus('❌ Please enter a valid radius');
        return;
    }
    
    try {
        const response = await fetch(apiUrl(config.searchEndpoint), {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                center: [viewX, viewY],
                radius: radius,
                x: viewX,
                y: viewY
            })
        });
        
        if (response.ok) {
            points = await response.json();
            updatePointsList();
            draw();
            setStatus('✓ Found ' + points.length + ' ' + config.pointLabel.toLowerCase() + (points.length !== 1 ? 's' : ''));
        } else {
            setStatus('❌ Search failed');
        }
    } catch (error) {
        setStatus('❌ Error: ' + error.message);
    }
}

async function loadAllPoints() {
    try {
        const response = await fetch(apiUrl(config.pointsEndpoint));
        if (response.ok) {
            points = await response.json();
            updatePointsList();
            draw();
            setStatus('✓ Loaded ' + points.length + ' ' + config.pointLabel.toLowerCase() + (points.length !== 1 ? 's' : ''));
        } else {
            setStatus('❌ Failed to load ' + config.pointLabel.toLowerCase() + 's');
        }
    } catch (error) {
        setStatus('❌ Error: ' + error.message);
    }
}

function focusPoint(point) {
    viewX = point.x;
    viewY = point.y;
    zoom = 2;
    draw();
    if (isMobile()) toggleSidebar();
}

function updatePointsList() {
    const list = document.getElementById('pointsList');
    const count = document.getElementById('pointCount');
    count.textContent = points.length;
    
    if (points.length === 0) {
        list.innerHTML = '<div class="info-item">No ' + config.pointLabel.toLowerCase() + 's to display</div>';
        return;
    }
    
    list.innerHTML = points.map(point => {
        const label = point.name || point.data || config.pointLabel;
        const deleteBtn = config.showDelete && !config.readOnly 
            ? '<button class="point-delete" onclick="event.stopPropagation(); deletePoint(\'' + point.id + '\')">Delete</button>'
            : '';
        return '<div class="point-item" onclick="focusPoint(' + JSON.stringify(point).replace(/"/g, "'") + ')">' +
            '<div class="point-info">' +
            '<div><strong>' + label + '</strong></div>' +
            '<div>X: ' + point.x.toFixed(2) + ', Y: ' + point.y.toFixed(2) + '</div>' +
            (point.radius ? '<div>Radius: ' + point.radius.toFixed(1) + '</div>' : '') +
            '</div>' +
            deleteBtn +
            '</div>';
    }).join('');
}

// Load points on startup
loadAllPoints();

// Auto-refresh every 2 seconds if configured
if (config.autoRefresh) {
    setInterval(loadAllPoints, config.refreshInterval || 2000);
}
