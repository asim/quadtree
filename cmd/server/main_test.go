package main

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/asim/quadtree"
)

func setupTestServer() {
	// Reset state
	center := quadtree.NewPoint(0.0, 0.0, nil)
	half := quadtree.NewPoint(90.0, 180.0, nil)
	bounds := quadtree.NewAABB(center, half)
	tree = quadtree.New(bounds, 0, nil)
	points = make(map[string]*quadtree.Point)
	pointToID = make(map[*quadtree.Point]string)
	idCounter = 0
}

func TestAddPoint(t *testing.T) {
	setupTestServer()

	reqBody := PointRequest{X: 10.5, Y: 20.3, Data: "Test Point"}
	body, _ := json.Marshal(reqBody)

	req := httptest.NewRequest(http.MethodPost, "/api/points", bytes.NewBuffer(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()

	handlePoints(w, req)

	if w.Code != http.StatusOK {
		t.Errorf("Expected status 200, got %d", w.Code)
	}

	var resp PointResponse
	json.NewDecoder(w.Body).Decode(&resp)

	if resp.ID != "1" {
		t.Errorf("Expected ID '1', got '%s'", resp.ID)
	}
	if resp.X != 10.5 {
		t.Errorf("Expected X 10.5, got %f", resp.X)
	}
	if resp.Y != 20.3 {
		t.Errorf("Expected Y 20.3, got %f", resp.Y)
	}
}

func TestGetAllPoints(t *testing.T) {
	setupTestServer()

	// Add some points
	p1 := quadtree.NewPoint(10.0, 20.0, "Point 1")
	p2 := quadtree.NewPoint(15.0, 25.0, "Point 2")
	tree.Insert(p1)
	tree.Insert(p2)
	points["1"] = p1
	points["2"] = p2
	pointToID[p1] = "1"
	pointToID[p2] = "2"
	idCounter = 2

	req := httptest.NewRequest(http.MethodGet, "/api/points", nil)
	w := httptest.NewRecorder()

	handlePoints(w, req)

	if w.Code != http.StatusOK {
		t.Errorf("Expected status 200, got %d", w.Code)
	}

	var resp []PointResponse
	json.NewDecoder(w.Body).Decode(&resp)

	if len(resp) != 2 {
		t.Errorf("Expected 2 points, got %d", len(resp))
	}
}

func TestGetPoint(t *testing.T) {
	setupTestServer()

	p := quadtree.NewPoint(10.0, 20.0, "Test Point")
	tree.Insert(p)
	points["1"] = p
	pointToID[p] = "1"
	idCounter = 1

	req := httptest.NewRequest(http.MethodGet, "/api/points/1", nil)
	w := httptest.NewRecorder()

	handlePointByID(w, req)

	if w.Code != http.StatusOK {
		t.Errorf("Expected status 200, got %d", w.Code)
	}

	var resp PointResponse
	json.NewDecoder(w.Body).Decode(&resp)

	if resp.ID != "1" {
		t.Errorf("Expected ID '1', got '%s'", resp.ID)
	}
}

func TestDeletePoint(t *testing.T) {
	setupTestServer()

	p := quadtree.NewPoint(10.0, 20.0, "Test Point")
	tree.Insert(p)
	points["1"] = p
	pointToID[p] = "1"
	idCounter = 1

	req := httptest.NewRequest(http.MethodDelete, "/api/points/1", nil)
	w := httptest.NewRecorder()

	handlePointByID(w, req)

	if w.Code != http.StatusNoContent {
		t.Errorf("Expected status 204, got %d", w.Code)
	}

	if len(points) != 0 {
		t.Errorf("Expected 0 points after deletion, got %d", len(points))
	}

	if len(pointToID) != 0 {
		t.Errorf("Expected 0 entries in pointToID after deletion, got %d", len(pointToID))
	}
}

func TestSearch(t *testing.T) {
	setupTestServer()

	// Add points
	p1 := quadtree.NewPoint(5.0, 5.0, "Point 1")
	p2 := quadtree.NewPoint(15.0, 15.0, "Point 2")
	p3 := quadtree.NewPoint(50.0, 50.0, "Point 3")
	tree.Insert(p1)
	tree.Insert(p2)
	tree.Insert(p3)
	points["1"] = p1
	points["2"] = p2
	points["3"] = p3
	pointToID[p1] = "1"
	pointToID[p2] = "2"
	pointToID[p3] = "3"
	idCounter = 3

	// Search for points near origin
	searchReq := SearchRequest{
		Center: []float64{0.0, 0.0},
		Radius: 10.0,
	}
	body, _ := json.Marshal(searchReq)

	req := httptest.NewRequest(http.MethodPost, "/api/search", bytes.NewBuffer(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()

	handleSearch(w, req)

	if w.Code != http.StatusOK {
		t.Errorf("Expected status 200, got %d", w.Code)
	}

	var resp []PointResponse
	json.NewDecoder(w.Body).Decode(&resp)

	// Should find only Point 1 (5,5) within the search box
	if len(resp) != 1 {
		t.Errorf("Expected 1 point in search results, got %d", len(resp))
	}

	if len(resp) > 0 && resp[0].Data != "Point 1" {
		t.Errorf("Expected to find 'Point 1', got '%v'", resp[0].Data)
	}
}

func TestUpdatePoint(t *testing.T) {
	setupTestServer()

	p := quadtree.NewPoint(10.0, 20.0, "Original")
	tree.Insert(p)
	points["1"] = p
	pointToID[p] = "1"
	idCounter = 1

	updateReq := PointRequest{X: 15.0, Y: 25.0, Data: "Updated"}
	body, _ := json.Marshal(updateReq)

	req := httptest.NewRequest(http.MethodPut, "/api/points/1", bytes.NewBuffer(body))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()

	handlePointByID(w, req)

	if w.Code != http.StatusOK {
		t.Errorf("Expected status 200, got %d", w.Code)
	}

	var resp PointResponse
	json.NewDecoder(w.Body).Decode(&resp)

	if resp.X != 15.0 || resp.Y != 25.0 {
		t.Errorf("Expected coordinates (15.0, 25.0), got (%f, %f)", resp.X, resp.Y)
	}

	if resp.Data != "Updated" {
		t.Errorf("Expected data 'Updated', got '%v'", resp.Data)
	}
}

func TestServeIndex(t *testing.T) {
	req := httptest.NewRequest(http.MethodGet, "/", nil)
	w := httptest.NewRecorder()

	serveIndex(w, req)

	if w.Code != http.StatusOK {
		t.Errorf("Expected status 200, got %d", w.Code)
	}

	contentType := w.Header().Get("Content-Type")
	if contentType != "text/html" {
		t.Errorf("Expected Content-Type 'text/html', got '%s'", contentType)
	}

	body := w.Body.String()
	if len(body) == 0 {
		t.Error("Expected non-empty HTML body")
	}
}
