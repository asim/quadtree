package quadtree

import (
	"os"
	"testing"
)

func TestMemoryStore(t *testing.T) {
	store := NewMemoryStore()
	defer store.Close()
	
	// Test Save
	point1 := NewPoint(10.0, 20.0, "Test Point 1")
	if err := store.Save("1", point1); err != nil {
		t.Fatalf("Failed to save point: %v", err)
	}
	
	point2 := NewPoint(30.0, 40.0, "Test Point 2")
	if err := store.Save("2", point2); err != nil {
		t.Fatalf("Failed to save point: %v", err)
	}
	
	// Test Load
	loaded, err := store.Load("1")
	if err != nil {
		t.Fatalf("Failed to load point: %v", err)
	}
	if loaded == nil {
		t.Fatal("Expected point to be loaded")
	}
	x, y := loaded.Coordinates()
	if x != 10.0 || y != 20.0 {
		t.Errorf("Expected coordinates (10, 20), got (%.1f, %.1f)", x, y)
	}
	
	// Test List
	points, err := store.List()
	if err != nil {
		t.Fatalf("Failed to list points: %v", err)
	}
	if len(points) != 2 {
		t.Errorf("Expected 2 points, got %d", len(points))
	}
	
	// Test Delete
	if err := store.Delete("1"); err != nil {
		t.Fatalf("Failed to delete point: %v", err)
	}
	
	loaded, err = store.Load("1")
	if err != nil {
		t.Fatalf("Failed to load after delete: %v", err)
	}
	if loaded != nil {
		t.Error("Expected point to be deleted")
	}
	
	points, err = store.List()
	if err != nil {
		t.Fatalf("Failed to list after delete: %v", err)
	}
	if len(points) != 1 {
		t.Errorf("Expected 1 point after delete, got %d", len(points))
	}
}

func TestFileStore(t *testing.T) {
	filename := "/tmp/quadtree_test.json"
	defer os.Remove(filename)
	
	// Test new file store
	store, err := NewFileStore(filename)
	if err != nil {
		t.Fatalf("Failed to create file store: %v", err)
	}
	
	// Test Save
	point1 := NewPoint(10.0, 20.0, "Test Point 1")
	if err := store.Save("1", point1); err != nil {
		t.Fatalf("Failed to save point: %v", err)
	}
	
	point2 := NewPoint(30.0, 40.0, "Test Point 2")
	if err := store.Save("2", point2); err != nil {
		t.Fatalf("Failed to save point: %v", err)
	}
	
	// Close and reload
	if err := store.Close(); err != nil {
		t.Fatalf("Failed to close store: %v", err)
	}
	
	store, err = NewFileStore(filename)
	if err != nil {
		t.Fatalf("Failed to reload file store: %v", err)
	}
	defer store.Close()
	
	// Test Load after reload
	loaded, err := store.Load("1")
	if err != nil {
		t.Fatalf("Failed to load point: %v", err)
	}
	if loaded == nil {
		t.Fatal("Expected point to be loaded after reload")
	}
	x, y := loaded.Coordinates()
	if x != 10.0 || y != 20.0 {
		t.Errorf("Expected coordinates (10, 20), got (%.1f, %.1f)", x, y)
	}
	
	// Test List
	points, err := store.List()
	if err != nil {
		t.Fatalf("Failed to list points: %v", err)
	}
	if len(points) != 2 {
		t.Errorf("Expected 2 points, got %d", len(points))
	}
	
	// Test Delete
	if err := store.Delete("1"); err != nil {
		t.Fatalf("Failed to delete point: %v", err)
	}
	
	loaded, err = store.Load("1")
	if err != nil {
		t.Fatalf("Failed to load after delete: %v", err)
	}
	if loaded != nil {
		t.Error("Expected point to be deleted")
	}
	
	points, err = store.List()
	if err != nil {
		t.Fatalf("Failed to list after delete: %v", err)
	}
	if len(points) != 1 {
		t.Errorf("Expected 1 point after delete, got %d", len(points))
	}
}

func TestFileStoreDataPreservation(t *testing.T) {
	filename := "/tmp/quadtree_test_data.json"
	defer os.Remove(filename)
	
	store, err := NewFileStore(filename)
	if err != nil {
		t.Fatalf("Failed to create file store: %v", err)
	}
	
	// Save point with complex data
	data := map[string]interface{}{
		"name": "Berlin",
		"country": "Germany",
		"population": 3520000,
	}
	point := NewPoint(52.5200, 13.4050, data)
	if err := store.Save("berlin", point); err != nil {
		t.Fatalf("Failed to save point: %v", err)
	}
	
	if err := store.Close(); err != nil {
		t.Fatalf("Failed to close store: %v", err)
	}
	
	// Reload and verify data
	store, err = NewFileStore(filename)
	if err != nil {
		t.Fatalf("Failed to reload file store: %v", err)
	}
	defer store.Close()
	
	loaded, err := store.Load("berlin")
	if err != nil {
		t.Fatalf("Failed to load point: %v", err)
	}
	
	loadedData, ok := loaded.Data().(map[string]interface{})
	if !ok {
		t.Fatal("Expected data to be a map")
	}
	
	if loadedData["name"] != "Berlin" {
		t.Errorf("Expected name 'Berlin', got '%v'", loadedData["name"])
	}
}
