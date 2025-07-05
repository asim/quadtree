package quadtree

import (
	"testing"
	"math"
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
