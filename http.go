package quadtree

import (
	"encoding/json"
	"net/http"
	"strings"
	"sync"
)

// HTTPHandler provides HTTP API for a quadtree with persistence
type HTTPHandler struct {
	tree      *QuadTree
	store     Store
	points    map[string]*Point
	pointToID map[*Point]string
	mu        sync.RWMutex
	idCounter int
}

// PointRequest is the JSON request format for adding/updating points
type PointRequest struct {
	X    float64     `json:"x"`
	Y    float64     `json:"y"`
	Data interface{} `json:"data"`
}

// PointResponse is the JSON response format for points
type PointResponse struct {
	ID   string      `json:"id"`
	X    float64     `json:"x"`
	Y    float64     `json:"y"`
	Data interface{} `json:"data"`
}

// SearchRequest is the JSON request format for search
type SearchRequest struct {
	Center []float64 `json:"center"`
	Radius float64   `json:"radius"`
}

// NewHTTPHandler creates an HTTP handler for the given quadtree and store
func NewHTTPHandler(tree *QuadTree, store Store) *HTTPHandler {
	h := &HTTPHandler{
		tree:      tree,
		store:     store,
		points:    make(map[string]*Point),
		pointToID: make(map[*Point]string),
	}

	// Load existing points from store
	if store != nil {
		h.loadFromStore()
	}

	return h
}

func (h *HTTPHandler) loadFromStore() {
	storedPoints, err := h.store.List()
	if err != nil {
		return
	}

	h.mu.Lock()
	defer h.mu.Unlock()

	for id, point := range storedPoints {
		if h.tree.Insert(point) {
			h.points[id] = point
			h.pointToID[point] = id
		}
	}
}

// ServeHTTP handles /points and /points/{id} and /search
func (h *HTTPHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	path := r.URL.Path

	if path == "/points" || path == "/points/" {
		h.handlePoints(w, r)
		return
	}

	if strings.HasPrefix(path, "/points/") {
		id := strings.TrimPrefix(path, "/points/")
		h.handlePointByID(w, r, id)
		return
	}

	if path == "/search" {
		h.handleSearch(w, r)
		return
	}

	http.NotFound(w, r)
}

func (h *HTTPHandler) handlePoints(w http.ResponseWriter, r *http.Request) {
	switch r.Method {
	case http.MethodPost:
		h.addPoint(w, r)
	case http.MethodGet:
		h.getAllPoints(w, r)
	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func (h *HTTPHandler) addPoint(w http.ResponseWriter, r *http.Request) {
	var req PointRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	h.mu.Lock()
	h.idCounter++
	id := string(rune('0' + h.idCounter%10)) // Simple ID generation
	for i := h.idCounter / 10; i > 0; i /= 10 {
		id = string(rune('0'+i%10)) + id
	}

	point := NewPoint(req.X, req.Y, req.Data)

	if !h.tree.Insert(point) {
		h.mu.Unlock()
		http.Error(w, "Failed to insert point", http.StatusBadRequest)
		return
	}

	h.points[id] = point
	h.pointToID[point] = id

	if h.store != nil {
		h.store.Save(id, point)
	}
	h.mu.Unlock()

	resp := PointResponse{ID: id, X: req.X, Y: req.Y, Data: req.Data}
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(resp)
}

func (h *HTTPHandler) getAllPoints(w http.ResponseWriter, r *http.Request) {
	h.mu.RLock()
	defer h.mu.RUnlock()

	responses := make([]PointResponse, 0, len(h.points))
	for id, point := range h.points {
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

func (h *HTTPHandler) handlePointByID(w http.ResponseWriter, r *http.Request, id string) {
	switch r.Method {
	case http.MethodGet:
		h.getPoint(w, id)
	case http.MethodDelete:
		h.deletePoint(w, id)
	default:
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
	}
}

func (h *HTTPHandler) getPoint(w http.ResponseWriter, id string) {
	h.mu.RLock()
	point, exists := h.points[id]
	h.mu.RUnlock()

	if !exists {
		http.Error(w, "Point not found", http.StatusNotFound)
		return
	}

	x, y := point.Coordinates()
	resp := PointResponse{ID: id, X: x, Y: y, Data: point.Data()}
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(resp)
}

func (h *HTTPHandler) deletePoint(w http.ResponseWriter, id string) {
	h.mu.Lock()
	defer h.mu.Unlock()

	point, exists := h.points[id]
	if !exists {
		http.Error(w, "Point not found", http.StatusNotFound)
		return
	}

	h.tree.Remove(point)
	delete(h.points, id)
	delete(h.pointToID, point)

	if h.store != nil {
		h.store.Delete(id)
	}

	w.WriteHeader(http.StatusNoContent)
}

func (h *HTTPHandler) handleSearch(w http.ResponseWriter, r *http.Request) {
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
		http.Error(w, "Center must be [x, y]", http.StatusBadRequest)
		return
	}

	h.mu.RLock()
	center := NewPoint(req.Center[0], req.Center[1], nil)
	half := NewPoint(req.Radius, req.Radius, nil)
	bounds := NewAABB(center, half)
	results := h.tree.Search(bounds)

	responses := make([]PointResponse, 0, len(results))
	for _, point := range results {
		x, y := point.Coordinates()
		responses = append(responses, PointResponse{
			ID:   h.pointToID[point],
			X:    x,
			Y:    y,
			Data: point.Data(),
		})
	}
	h.mu.RUnlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(responses)
}

// Points returns all points (for external iteration)
func (h *HTTPHandler) Points() map[string]*Point {
	h.mu.RLock()
	defer h.mu.RUnlock()

	result := make(map[string]*Point, len(h.points))
	for k, v := range h.points {
		result[k] = v
	}
	return result
}
