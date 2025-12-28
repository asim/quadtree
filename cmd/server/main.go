package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"sync"
	"syscall"

	"github.com/asim/quadtree"
)

var (
	tree        *quadtree.QuadTree
	points      = make(map[string]*quadtree.Point)
	pointToID   = make(map[*quadtree.Point]string)
	mu          sync.RWMutex
	idCounter   int
	store       quadtree.Store
)

type PointRequest struct {
	X    float64     `json:"x"`
	Y    float64     `json:"y"`
	Data interface{} `json:"data"`
}

type PointResponse struct {
	ID   string      `json:"id"`
	X    float64     `json:"x"`
	Y    float64     `json:"y"`
	Data interface{} `json:"data"`
}

type SearchRequest struct {
	Center []float64 `json:"center"`
	Radius float64   `json:"radius"`
}

func init() {
	// Initialize quadtree with world bounds
	center := quadtree.NewPoint(0.0, 0.0, nil)
	half := quadtree.NewPoint(90.0, 180.0, nil)
	bounds := quadtree.NewAABB(center, half)
	tree = quadtree.New(bounds, 0, nil)
}

func main() {
	// Parse command-line flags
	storeType := flag.String("store", "memory", "Store type: memory or file")
	storeFile := flag.String("store-file", "quadtree.json", "File path for file store")
	flag.Parse()

	// Initialize store based on type
	var err error
	switch *storeType {
	case "file":
		store, err = quadtree.NewFileStore(*storeFile)
		if err != nil {
			log.Fatalf("Failed to initialize file store: %v", err)
		}
		log.Printf("Using file store: %s", *storeFile)
	case "memory":
		store = quadtree.NewMemoryStore()
		log.Printf("Using memory store")
	default:
		log.Fatalf("Unknown store type: %s", *storeType)
	}

	// Load existing points from store
	if err := loadPointsFromStore(); err != nil {
		log.Fatalf("Failed to load points from store: %v", err)
	}

	// Handle graceful shutdown
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, os.Interrupt, syscall.SIGTERM)
	go func() {
		<-sigChan
		log.Println("Shutting down server...")
		if err := store.Close(); err != nil {
			log.Printf("Error closing store: %v", err)
		}
		os.Exit(0)
	}()

	// API endpoints
	http.HandleFunc("/api/points", handlePoints)
	http.HandleFunc("/api/points/", handlePointByID)
	http.HandleFunc("/api/search", handleSearch)

	// Serve static files
	http.HandleFunc("/", serveIndex)

	port := ":8080"
	log.Printf("Starting server on %s", port)
	log.Fatal(http.ListenAndServe(port, nil))
}

// loadPointsFromStore loads all points from the store and rebuilds the quadtree
func loadPointsFromStore() error {
	storedPoints, err := store.List()
	if err != nil {
		return err
	}

	mu.Lock()
	defer mu.Unlock()

	for id, point := range storedPoints {
		if tree.Insert(point) {
			points[id] = point
			pointToID[point] = id
			
			// Update idCounter to be higher than any loaded ID
			if idNum, err := strconv.Atoi(id); err == nil && idNum > idCounter {
				idCounter = idNum
			}
		}
	}

	log.Printf("Loaded %d points from store", len(storedPoints))
	return nil
}

func handlePoints(w http.ResponseWriter, r *http.Request) {
	switch r.Method {
	case http.MethodPost:
		addPoint(w, r)
	case http.MethodGet:
		getAllPoints(w, r)
	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func addPoint(w http.ResponseWriter, r *http.Request) {
	var req PointRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	mu.Lock()
	idCounter++
	id := strconv.Itoa(idCounter)
	point := quadtree.NewPoint(req.X, req.Y, req.Data)
	
	if !tree.Insert(point) {
		mu.Unlock()
		http.Error(w, "Failed to insert point", http.StatusBadRequest)
		return
	}
	
	points[id] = point
	pointToID[point] = id
	
	// Save to store
	if err := store.Save(id, point); err != nil {
		log.Printf("Warning: Failed to save point to store: %v", err)
	}
	mu.Unlock()

	resp := PointResponse{
		ID:   id,
		X:    req.X,
		Y:    req.Y,
		Data: req.Data,
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(resp)
}

func getAllPoints(w http.ResponseWriter, r *http.Request) {
	mu.RLock()
	defer mu.RUnlock()

	responses := make([]PointResponse, 0)
	for id, point := range points {
		x, y := point.Coordinates()
		responses = append(responses, PointResponse{
			ID:   id,
			X:    x,
			Y:    y,
			Data: point.Data(),
		})
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(responses)
}

func handlePointByID(w http.ResponseWriter, r *http.Request) {
	// Extract ID from path
	id := r.URL.Path[len("/api/points/"):]
	if id == "" {
		http.Error(w, "ID required", http.StatusBadRequest)
		return
	}

	switch r.Method {
	case http.MethodGet:
		getPoint(w, r, id)
	case http.MethodPut:
		updatePoint(w, r, id)
	case http.MethodDelete:
		deletePoint(w, r, id)
	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func getPoint(w http.ResponseWriter, r *http.Request, id string) {
	mu.RLock()
	point, exists := points[id]
	mu.RUnlock()

	if !exists {
		http.Error(w, "Point not found", http.StatusNotFound)
		return
	}

	x, y := point.Coordinates()
	resp := PointResponse{
		ID:   id,
		X:    x,
		Y:    y,
		Data: point.Data(),
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(resp)
}

func updatePoint(w http.ResponseWriter, r *http.Request, id string) {
	var req PointRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	mu.Lock()
	defer mu.Unlock()

	oldPoint, exists := points[id]
	if !exists {
		http.Error(w, "Point not found", http.StatusNotFound)
		return
	}

	// Remove old point from tree and reverse map
	if !tree.Remove(oldPoint) {
		http.Error(w, "Failed to remove old point", http.StatusInternalServerError)
		return
	}
	delete(pointToID, oldPoint)

	// Create and insert new point with updated coordinates and data
	newPoint := quadtree.NewPoint(req.X, req.Y, req.Data)
	if !tree.Insert(newPoint) {
		// Try to restore old point if new insertion fails
		tree.Insert(oldPoint)
		pointToID[oldPoint] = id
		http.Error(w, "Failed to insert updated point", http.StatusBadRequest)
		return
	}

	// Update mappings
	points[id] = newPoint
	pointToID[newPoint] = id
	
	// Save to store
	if err := store.Save(id, newPoint); err != nil {
		log.Printf("Warning: Failed to save updated point to store: %v", err)
	}

	resp := PointResponse{
		ID:   id,
		X:    req.X,
		Y:    req.Y,
		Data: req.Data,
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(resp)
}

func deletePoint(w http.ResponseWriter, r *http.Request, id string) {
	mu.Lock()
	defer mu.Unlock()

	point, exists := points[id]
	if !exists {
		http.Error(w, "Point not found", http.StatusNotFound)
		return
	}

	if !tree.Remove(point) {
		http.Error(w, "Failed to remove point", http.StatusInternalServerError)
		return
	}

	delete(points, id)
	delete(pointToID, point)
	
	// Delete from store
	if err := store.Delete(id); err != nil {
		log.Printf("Warning: Failed to delete point from store: %v", err)
	}
	
	w.WriteHeader(http.StatusNoContent)
}

func handleSearch(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req SearchRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	if len(req.Center) != 2 {
		http.Error(w, "Center must be an array of two values [x, y]", http.StatusBadRequest)
		return
	}

	mu.RLock()
	center := quadtree.NewPoint(req.Center[0], req.Center[1], nil)
	half := quadtree.NewPoint(req.Radius, req.Radius, nil)
	bounds := quadtree.NewAABB(center, half)
	results := tree.Search(bounds)

	responses := make([]PointResponse, 0)
	for _, point := range results {
		x, y := point.Coordinates()
		pointID := pointToID[point]
		responses = append(responses, PointResponse{
			ID:   pointID,
			X:    x,
			Y:    y,
			Data: point.Data(),
		})
	}
	mu.RUnlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(responses)
}

func serveIndex(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		http.NotFound(w, r)
		return
	}

	html := `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>QuadTree Viewer</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #ffffff;
            color: #333;
            height: 100vh;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }
        header {
            background: #f5f5f5;
            padding: 0.75rem;
            border-bottom: 2px solid #ddd;
        }
        h1 { font-size: 1.25rem; margin-bottom: 0.5rem; color: #333; }
        .controls {
            display: flex;
            gap: 0.5rem;
            flex-wrap: wrap;
            align-items: center;
            margin-top: 0.5rem;
        }
        .control-group {
            display: flex;
            align-items: center;
            gap: 0.3rem;
        }
        input, button {
            padding: 0.4rem 0.5rem;
            border-radius: 4px;
            border: 1px solid #ccc;
            background: #fff;
            color: #333;
            font-size: 0.85rem;
            min-height: 36px;
        }
        input {
            width: 60px;
        }
        input[type="text"] {
            width: 100px;
        }
        button {
            cursor: pointer;
            background: #0066cc;
            border-color: #0066cc;
            color: #fff;
            white-space: nowrap;
        }
        button:hover, button:active { background: #0052a3; }
        .main-content {
            display: flex;
            flex: 1;
            overflow: hidden;
        }
        .canvas-container {
            flex: 1;
            position: relative;
            overflow: hidden;
            min-height: 300px;
        }
        canvas {
            display: block;
            cursor: move;
            background: #ffffff;
            touch-action: none;
        }
        .sidebar {
            width: 300px;
            background: #f5f5f5;
            border-left: 2px solid #ddd;
            padding: 1rem;
            overflow-y: auto;
        }
        .info-section {
            margin-bottom: 1.5rem;
        }
        .info-section h3 {
            font-size: 1rem;
            margin-bottom: 0.5rem;
            color: #0066cc;
        }
        .info-item {
            background: #fff;
            padding: 0.5rem;
            margin-bottom: 0.5rem;
            border-radius: 4px;
            font-size: 0.85rem;
            border: 1px solid #e0e0e0;
        }
        .point-item {
            background: #fff;
            padding: 0.75rem;
            margin-bottom: 0.5rem;
            border-radius: 4px;
            font-size: 0.85rem;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border: 1px solid #e0e0e0;
        }
        .point-info { flex: 1; }
        .point-delete {
            background: #cc0000;
            border: none;
            padding: 0.4rem 0.6rem;
            font-size: 0.75rem;
            color: #fff;
            min-height: 36px;
        }
        .status {
            position: fixed;
            bottom: 1rem;
            right: 1rem;
            background: #f5f5f5;
            padding: 0.5rem 1rem;
            border-radius: 4px;
            border: 1px solid #ddd;
            font-size: 0.85rem;
            z-index: 1000;
            display: none;
        }
        .status.visible {
            display: block;
        }
        
        /* Mobile Responsive Styles */
        @media (max-width: 768px) {
            header {
                padding: 0.75rem;
            }
            h1 { 
                font-size: 1.2rem; 
                margin-bottom: 0.5rem;
            }
            .controls {
                gap: 0.5rem;
            }
            .control-group {
                flex: 1 1 auto;
                min-width: 140px;
            }
            .control-group label {
                font-size: 0.85rem;
            }
            input, button {
                font-size: 0.85rem;
                padding: 0.5rem;
            }
            input {
                width: 70px;
                flex: 1;
            }
            input[type="text"] {
                width: 100%;
            }
            button {
                flex: 1 1 auto;
                min-width: 100px;
            }
            .main-content {
                flex-direction: column;
            }
            .canvas-container {
                min-height: 50vh;
                flex: 0 0 50vh;
            }
            .sidebar {
                width: 100%;
                border-left: none;
                border-top: 2px solid #ddd;
                flex: 1;
                overflow-y: auto;
            }
            .status {
                bottom: 0.5rem;
                right: 0.5rem;
                left: 0.5rem;
                font-size: 0.75rem;
                padding: 0.4rem 0.6rem;
                text-align: center;
            }
        }
        
        @media (max-width: 480px) {
            h1 {
                font-size: 1rem;
            }
            .control-group {
                flex: 1 1 100%;
            }
            button {
                width: 100%;
            }
            .canvas-container {
                min-height: 40vh;
                flex: 0 0 40vh;
            }
        }
    </style>
</head>
<body>
    <header>
        <h1>🗺️ QuadTree Viewer</h1>
        <div class="controls">
            <div class="control-group">
                <label>X:</label>
                <input type="number" id="addX" step="any" value="0">
            </div>
            <div class="control-group">
                <label>Y:</label>
                <input type="number" id="addY" step="any" value="0">
            </div>
            <div class="control-group">
                <label>Label:</label>
                <input type="text" id="addData" placeholder="Point label">
            </div>
            <button onclick="addPoint()">Add Point</button>
            
            <div class="control-group" style="margin-left: 2rem;">
                <label>Search Size:</label>
                <input type="number" id="searchRadius" value="10" step="1" min="1">
            </div>
            <button onclick="search()">Search Region</button>
            <button onclick="loadAllPoints()">Reload All</button>
        </div>
    </header>
    
    <div class="main-content">
        <div class="canvas-container">
            <canvas id="canvas"></canvas>
        </div>
        
        <div class="sidebar">
            <div class="info-section">
                <h3>Current Location</h3>
                <div class="info-item">
                    <div>X: <span id="currentX">0.00</span></div>
                    <div>Y: <span id="currentY">0.00</span></div>
                    <div>Zoom: <span id="currentZoom">1.00</span>x</div>
                </div>
            </div>
            
            <div class="info-section">
                <h3>Visible Points (<span id="pointCount">0</span>)</h3>
                <div id="pointsList"></div>
            </div>
        </div>
    </div>
    
    <div class="status" id="status">Ready</div>

    <script>
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        
        let viewX = 0;
        let viewY = 0;
        let zoom = 1;
        let points = [];
        let isDragging = false;
        let lastMouseX = 0;
        let lastMouseY = 0;
        
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
            ctx.strokeStyle = '#e0e0e0';
            ctx.lineWidth = 1;
            
            const gridSize = 10 / zoom;
            const startX = Math.floor((viewX - canvas.width / 2 / zoom) / gridSize) * gridSize;
            const startY = Math.floor((viewY - canvas.height / 2 / zoom) / gridSize) * gridSize;
            const endX = viewX + canvas.width / 2 / zoom;
            const endY = viewY + canvas.height / 2 / zoom;
            
            for (let x = startX; x <= endX; x += gridSize) {
                const screen = worldToScreen(x, 0);
                ctx.beginPath();
                ctx.moveTo(screen.x, 0);
                ctx.lineTo(screen.x, canvas.height);
                ctx.stroke();
            }
            
            for (let y = startY; y <= endY; y += gridSize) {
                const screen = worldToScreen(0, y);
                ctx.beginPath();
                ctx.moveTo(0, screen.y);
                ctx.lineTo(canvas.width, screen.y);
                ctx.stroke();
            }
            
            // Draw axes
            ctx.strokeStyle = '#999';
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
                    
                    ctx.fillStyle = '#00cc66';
                    ctx.beginPath();
                    ctx.arc(screen.x, screen.y, 5, 0, 2 * Math.PI);
                    ctx.fill();
                    
                    if (point.data) {
                        ctx.fillStyle = '#333';
                        ctx.font = '12px sans-serif';
                        ctx.fillText(point.data, screen.x + 8, screen.y - 8);
                    }
                }
            });
            
            // Update info
            document.getElementById('currentX').textContent = viewX.toFixed(2);
            document.getElementById('currentY').textContent = viewY.toFixed(2);
            document.getElementById('currentZoom').textContent = zoom.toFixed(2);
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
            const step = 10 / zoom;
            switch(e.key) {
                case 'ArrowUp':
                    viewY += step;
                    draw();
                    break;
                case 'ArrowDown':
                    viewY -= step;
                    draw();
                    break;
                case 'ArrowLeft':
                    viewX -= step;
                    draw();
                    break;
                case 'ArrowRight':
                    viewX += step;
                    draw();
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
        
        async function addPoint() {
            const x = parseFloat(document.getElementById('addX').value);
            const y = parseFloat(document.getElementById('addY').value);
            const data = document.getElementById('addData').value;
            
            if (isNaN(x) || isNaN(y)) {
                setStatus('❌ Invalid coordinates');
                return;
            }
            
            try {
                const response = await fetch('/api/points', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ x, y, data: data || null })
                });
                
                if (response.ok) {
                    setStatus('✓ Point added');
                    loadAllPoints();
                } else {
                    setStatus('❌ Failed to add point');
                }
            } catch (error) {
                setStatus('❌ Error: ' + error.message);
            }
        }
        
        async function deletePoint(id) {
            try {
                const response = await fetch('/api/points/' + id, {
                    method: 'DELETE'
                });
                
                if (response.ok) {
                    setStatus('✓ Point deleted');
                    loadAllPoints();
                } else {
                    setStatus('❌ Failed to delete point');
                }
            } catch (error) {
                setStatus('❌ Error: ' + error.message);
            }
        }
        
        async function search() {
            const radius = parseFloat(document.getElementById('searchRadius').value);
            
            try {
                const response = await fetch('/api/search', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        center: [viewX, viewY],
                        radius: radius
                    })
                });
                
                if (response.ok) {
                    points = await response.json();
                    updatePointsList();
                    draw();
                    setStatus('✓ Found ' + points.length + ' points');
                } else {
                    setStatus('❌ Search failed');
                }
            } catch (error) {
                setStatus('❌ Error: ' + error.message);
            }
        }
        
        async function loadAllPoints() {
            try {
                const response = await fetch('/api/points');
                if (response.ok) {
                    points = await response.json();
                    updatePointsList();
                    draw();
                    setStatus('✓ Loaded ' + points.length + ' points');
                } else {
                    setStatus('❌ Failed to load points');
                }
            } catch (error) {
                setStatus('❌ Error: ' + error.message);
            }
        }
        
        function updatePointsList() {
            const list = document.getElementById('pointsList');
            const count = document.getElementById('pointCount');
            count.textContent = points.length;
            
            if (points.length === 0) {
                list.innerHTML = '<div class="info-item">No points to display</div>';
                return;
            }
            
            list.innerHTML = points.map(point => {
                return '<div class="point-item">' +
                    '<div class="point-info">' +
                    '<div><strong>' + (point.data || 'Point') + '</strong></div>' +
                    '<div>X: ' + point.x.toFixed(2) + ', Y: ' + point.y.toFixed(2) + '</div>' +
                    '</div>' +
                    '<button class="point-delete" onclick="deletePoint(\'' + point.id + '\')">Delete</button>' +
                    '</div>';
            }).join('');
        }
        
        // Load points on startup
        loadAllPoints();
    </script>
</body>
</html>`

	w.Header().Set("Content-Type", "text/html")
	fmt.Fprint(w, html)
}
