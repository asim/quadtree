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
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'Roboto', 'Oxygen', 'Ubuntu', 'Cantarell', sans-serif;
            background: #f8f9fa;
            color: #1a202c;
            height: 100vh;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }
        .hamburger {
            display: none;
            background: transparent;
            border: none;
            color: #4a5568;
            font-size: 1.25rem;
            width: 36px;
            height: 36px;
            border-radius: 6px;
            cursor: pointer;
            align-items: center;
            justify-content: center;
            transition: all 0.2s;
        }
        .hamburger:hover {
            background: #e2e8f0;
            color: #1a202c;
        }
        header {
            background: #ffffff;
            padding: 1rem 1.5rem;
            border-bottom: 1px solid #e2e8f0;
            box-shadow: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
        }
        .header-top {
            display: flex;
            align-items: center;
            gap: 0.75rem;
            margin-bottom: 0.75rem;
        }
        h1 { 
            font-size: 1.125rem; 
            font-weight: 600;
            margin: 0; 
            color: #1a202c; 
            flex: 1;
            letter-spacing: -0.01em;
        }
        .controls {
            display: flex;
            gap: 0.5rem;
            flex-wrap: wrap;
            align-items: center;
        }
        .control-group {
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }
        .control-group span {
            font-size: 0.875rem;
            color: #718096;
            font-weight: 500;
        }
        .xy-inputs {
            display: flex;
            gap: 0.5rem;
        }
        input, button {
            padding: 0.5rem 0.75rem;
            border-radius: 6px;
            border: 1px solid #e2e8f0;
            background: #ffffff;
            color: #1a202c;
            font-size: 0.875rem;
            min-height: 38px;
            transition: all 0.2s;
        }
        input:focus {
            outline: none;
            border-color: #4299e1;
            box-shadow: 0 0 0 3px rgba(66, 153, 225, 0.1);
        }
        input::placeholder {
            color: #a0aec0;
        }
        input {
            width: 70px;
        }
        input[type="text"] {
            width: 140px;
        }
        #searchRadius {
            width: 80px;
        }
        button {
            cursor: pointer;
            background: #ffffff;
            border: 1px solid #e2e8f0;
            color: #4a5568;
            white-space: nowrap;
            font-weight: 500;
            padding: 0.5rem 1rem;
        }
        button:hover {
            background: #f7fafc;
            border-color: #cbd5e0;
        }
        button:active {
            background: #edf2f7;
        }
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
            background: #ffffff;
        }
        canvas {
            display: block;
            cursor: move;
            background: #ffffff;
            touch-action: none;
        }
        .sidebar {
            width: 320px;
            background: #ffffff;
            border-left: 1px solid #e2e8f0;
            padding: 1.5rem;
            overflow-y: auto;
        }
        .sidebar-header {
            display: none;
        }
        .close-sidebar {
            display: none;
        }
        .info-section {
            margin-bottom: 1.5rem;
        }
        .info-section h3 {
            font-size: 0.75rem;
            margin-bottom: 0.75rem;
            color: #718096;
            text-transform: uppercase;
            font-weight: 600;
            letter-spacing: 0.05em;
        }
        .info-item {
            background: #f7fafc;
            padding: 0.75rem;
            margin-bottom: 0.5rem;
            border-radius: 6px;
            font-size: 0.875rem;
            border: 1px solid #e2e8f0;
        }
        .info-item div {
            line-height: 1.6;
            color: #4a5568;
        }
        .point-item {
            background: #ffffff;
            padding: 0.75rem;
            margin-bottom: 0.5rem;
            border-radius: 6px;
            font-size: 0.875rem;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border: 1px solid #e2e8f0;
            transition: all 0.2s;
        }
        .point-item:hover {
            border-color: #cbd5e0;
            box-shadow: 0 1px 3px 0 rgba(0, 0, 0, 0.1);
        }
        .point-info { 
            flex: 1;
            color: #4a5568;
        }
        .point-info strong {
            color: #1a202c;
            font-weight: 600;
        }
        .point-delete {
            background: #ffffff;
            border: 1px solid #e2e8f0;
            padding: 0.375rem 0.75rem;
            font-size: 0.75rem;
            color: #e53e3e;
            min-height: 32px;
            font-weight: 500;
        }
        .point-delete:hover {
            background: #fff5f5;
            border-color: #fc8181;
            color: #c53030;
        }
        .status {
            position: fixed;
            bottom: 1rem;
            right: 1rem;
            background: #1a202c;
            color: #ffffff;
            padding: 0.75rem 1rem;
            border-radius: 6px;
            font-size: 0.875rem;
            z-index: 1000;
            display: none;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
        }
        .status.visible {
            display: block;
        }
        
        /* Mobile Responsive Styles */
        @media (max-width: 768px) {
            body {
                overflow: auto;
                background: #ffffff;
            }
            .hamburger {
                display: flex;
            }
            header {
                padding: 1rem;
            }
            .header-top {
                margin-bottom: 0;
            }
            h1 { 
                font-size: 1rem;
            }
            header .controls {
                display: none;
            }
            .main-content {
                flex-direction: column;
                overflow: visible;
                height: auto;
                min-height: 100vh;
            }
            .canvas-container {
                min-height: 60vh;
                height: 60vh;
                flex: none;
            }
            .sidebar {
                width: 100%;
                border-left: none;
                flex: none;
                position: fixed;
                top: 0;
                right: -100%;
                height: 100vh;
                z-index: 1500;
                transition: right 0.3s ease;
                box-shadow: -4px 0 6px -1px rgba(0, 0, 0, 0.1);
            }
            .sidebar.open {
                right: 0;
            }
            .sidebar-header {
                display: flex;
                justify-content: space-between;
                align-items: center;
                margin-bottom: 1.5rem;
                padding-bottom: 1rem;
                border-bottom: 1px solid #e2e8f0;
            }
            .sidebar-header h2 {
                font-size: 1rem;
                color: #1a202c;
                font-weight: 600;
            }
            .close-sidebar {
                display: flex;
                background: #ffffff;
                border: 1px solid #e2e8f0;
                color: #718096;
                font-size: 1.25rem;
                width: 32px;
                height: 32px;
                border-radius: 6px;
                cursor: pointer;
                align-items: center;
                justify-content: center;
            }
            .close-sidebar:hover {
                background: #fff5f5;
                border-color: #fc8181;
                color: #e53e3e;
            }
            .sidebar::before {
                content: '';
                display: block;
            }
            .sidebar .controls {
                display: flex;
                flex-direction: column;
                gap: 0.75rem;
                margin-bottom: 1.5rem;
                align-items: stretch;
            }
            .sidebar .control-group {
                flex-direction: column;
                align-items: stretch;
                width: 100%;
                gap: 0.5rem;
            }
            .sidebar .control-group span {
                font-size: 0.75rem;
                color: #718096;
                text-transform: uppercase;
                font-weight: 600;
                letter-spacing: 0.05em;
            }
            .sidebar .xy-inputs {
                display: flex;
                gap: 0.5rem;
            }
            .sidebar .xy-inputs input {
                flex: 1;
            }
            .sidebar input {
                width: 100%;
                font-size: 0.875rem;
                padding: 0.625rem 0.75rem;
            }
            .sidebar input[type="text"] {
                width: 100%;
            }
            .sidebar button {
                width: 100%;
                padding: 0.625rem 1rem;
                font-size: 0.875rem;
            }
            .status {
                bottom: 1rem;
                right: 1rem;
                left: 1rem;
                font-size: 0.875rem;
                padding: 0.75rem 1rem;
                text-align: center;
            }
        }
    </style>
</head>
<body>
    <header>
        <div class="header-top">
            <button class="hamburger" onclick="toggleSidebar()" aria-label="Toggle menu">☰</button>
            <h1>🗺️ QuadTree Viewer</h1>
        </div>
        <div class="controls">
            <div class="xy-inputs">
                <input type="number" id="addX" step="any" placeholder="X">
                <input type="number" id="addY" step="any" placeholder="Y">
            </div>
            <input type="text" id="addData" placeholder="Label">
            <button onclick="addPoint()">Add Point</button>
            
            <div class="control-group">
                <span>Search</span>
                <input type="number" id="searchRadius" step="1" min="1" placeholder="Radius">
            </div>
            <button onclick="search()">Search Region</button>
            <button onclick="loadAllPoints()">Reload All</button>
        </div>
    </header>
    
    <div class="main-content">
        <div class="canvas-container">
            <canvas id="canvas"></canvas>
        </div>
        
        <div class="sidebar" id="sidebar">
            <div class="sidebar-header">
                <h2>Controls & Points</h2>
                <button class="close-sidebar" onclick="toggleSidebar()" aria-label="Close menu">×</button>
            </div>
            
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
        
        function toggleSidebar() {
            const sidebar = document.getElementById('sidebar');
            sidebar.classList.toggle('open');
        }
        
        function isMobile() {
            return window.innerWidth <= 768;
        }
        
        function moveControlsToSidebar() {
            if (isMobile()) {
                const controls = document.querySelector('.controls');
                const sidebar = document.getElementById('sidebar');
                const sidebarHeader = sidebar.querySelector('.sidebar-header');
                
                // Only move if not already in sidebar
                if (controls && !sidebar.contains(controls)) {
                    sidebar.insertBefore(controls, sidebarHeader.nextSibling);
                }
            } else {
                const controls = document.querySelector('.controls');
                const header = document.querySelector('header');
                
                // Move back to header if in sidebar
                if (controls && !header.contains(controls)) {
                    header.appendChild(controls);
                }
            }
        }
        
        function resizeCanvas() {
            const container = canvas.parentElement;
            canvas.width = container.clientWidth;
            canvas.height = container.clientHeight;
            draw();
            moveControlsToSidebar();
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
            
            // Draw axes
            ctx.strokeStyle = '#e2e8f0';
            ctx.lineWidth = 1;
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
                    
                    ctx.fillStyle = '#4299e1';
                    ctx.beginPath();
                    ctx.arc(screen.x, screen.y, 5, 0, 2 * Math.PI);
                    ctx.fill();
                    
                    if (point.data) {
                        ctx.fillStyle = '#2d3748';
                        ctx.font = '12px -apple-system, BlinkMacSystemFont, sans-serif';
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
            const xValue = document.getElementById('addX').value;
            const yValue = document.getElementById('addY').value;
            const data = document.getElementById('addData').value;
            
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
            
            await addPointAPI(x, y, data);
            
            // Close sidebar on mobile after adding point
            if (isMobile()) {
                toggleSidebar();
            }
        }
        
        async function addPointAPI(x, y, data) {
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
            const radiusValue = document.getElementById('searchRadius').value;
            const radius = parseFloat(radiusValue);
            
            if (radiusValue === '' || isNaN(radius) || radius <= 0) {
                setStatus('❌ Please enter a valid radius');
                return;
            }
            
            await searchAPI(radius);
            
            // Close sidebar on mobile after search
            if (isMobile()) {
                toggleSidebar();
            }
        }
        
        async function searchAPI(radius) {
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
                    
                    // Close sidebar on mobile after loading
                    if (isMobile()) {
                        toggleSidebar();
                    }
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
