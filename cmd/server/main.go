package main

import (
	"encoding/json"
	"flag"
	"io/fs"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"sync"
	"syscall"

	"github.com/asim/quadtree"
	"github.com/asim/quadtree/ui"
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

	// Serve static assets (CSS, JS)
	staticFS, _ := fs.Sub(ui.FS(), ".")
	http.Handle("/quadtree.css", http.FileServer(http.FS(staticFS)))
	http.Handle("/quadtree.js", http.FileServer(http.FS(staticFS)))

	// Serve the UI
	http.Handle("/", ui.Handler(ui.DefaultConfig()))

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
