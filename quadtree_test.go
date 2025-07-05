package quadtree

import (
	"math"
	"testing"
)

func TestKNearestReturnsSortedByDistance(t *testing.T) {
	center := NewPoint(0, 0, nil)
	half := NewPoint(10, 10, nil)
	qt := New(NewAABB(center, half), 0, nil)

	// Insert points at various distances
	points := []*Point{
		NewPoint(1, 1, "a"),   // sqrt(2)
		NewPoint(2, 2, "b"),   // sqrt(8)
		NewPoint(3, 3, "c"),   // sqrt(18)
		NewPoint(-1, -1, "d"), // sqrt(2)
		NewPoint(0, 5, "e"),   // 5
		NewPoint(5, 0, "f"),   // 5
	}
	for _, p := range points {
		qt.Insert(p)
	}

	query := NewAABB(center, half)
	results := qt.KNearest(query, 4, nil)

	if len(results) != 4 {
		t.Fatalf("expected 4 results, got %d", len(results))
	}

	// Check that results are sorted by distance to center
	lastDist := -1.0
	for i, p := range results {
		dist := math.Hypot(p.x-center.x, p.y-center.y)
		if i > 0 && dist < lastDist {
			t.Errorf("results not sorted by distance: %v before %v", lastDist, dist)
		}
		lastDist = dist
	}

	// Optionally, check the actual order
	expectedOrder := []string{"a", "d", "b", "c"} // sqrt(2), sqrt(2), sqrt(8), sqrt(18)
	for i, label := range expectedOrder {
		if results[i].data != label {
			t.Errorf("expected %s at position %d, got %v", label, i, results[i].data)
		}
	}
}

func TestKNearestEdgeCases(t *testing.T) {
	center := NewPoint(0, 0, nil)
	half := NewPoint(10, 10, nil)
	qt := New(NewAABB(center, half), 0, nil)

	points := []*Point{
		NewPoint(1, 0, "a"),   // 1
		NewPoint(-1, 0, "b"),  // 1
		NewPoint(0, 1, "c"),   // 1
		NewPoint(0, -1, "d"),  // 1
		NewPoint(5, 5, "e"),   // sqrt(50)
	}
	for _, p := range points {
		qt.Insert(p)
	}

	query := NewAABB(center, half)

	// k > n
	results := qt.KNearest(query, 10, nil)
	if len(results) != len(points) {
		t.Errorf("expected %d results, got %d", len(points), len(results))
	}

	// k = 0
	results = qt.KNearest(query, 0, nil)
	if len(results) != 0 {
		t.Errorf("expected 0 results, got %d", len(results))
	}

	// Points at same distance: first four should be at distance 1
	results = qt.KNearest(query, 4, nil)
	for _, p := range results {
		dist := math.Hypot(p.x-center.x, p.y-center.y)
		if math.Abs(dist-1) > 1e-9 {
			t.Errorf("expected distance 1, got %v", dist)
		}
	}

	// Filter function: only return points with label "a" or "e"
	filterFn := func(p *Point) bool {
		return p.data == "a" || p.data == "e"
	}
	results = qt.KNearest(query, 2, filterFn)
	if len(results) != 2 {
		t.Errorf("expected 2 results, got %d", len(results))
	}
	labels := []string{results[0].data.(string), results[1].data.(string)}
	if !(labels[0] == "a" && labels[1] == "e" || labels[0] == "e" && labels[1] == "a") {
		t.Errorf("expected results to be 'a' and 'e', got %v", labels)
	}
}
