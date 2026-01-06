package quadtree

import (
	"testing"
)

// TestInsertRemoveInsert reproduces the bug where after Remove+Insert,
// KNearest can't find the point even though Insert succeeded
func TestInsertRemoveInsert(t *testing.T) {
	// Create tree with world boundaries (like malten)
	center := NewPoint(0, 0, nil)
	half := NewPoint(90, 180, nil)
	boundary := NewAABB(center, half)
	tree := New(boundary, 0, nil)

	// Insert many points to force tree subdivision
	// (Capacity is 8, so 100 points will definitely subdivide)
	for i := 0; i < 100; i++ {
		lat := 51.0 + float64(i)*0.01
		lon := -0.3 + float64(i)*0.001
		p := NewPoint(lat, lon, i)
		if !tree.Insert(p) {
			t.Fatalf("Failed to insert point %d at %.4f, %.4f", i, lat, lon)
		}
	}

	t.Logf("Inserted 100 points, tree count: %d", tree.Count())

	// Insert a point at specific location (like Hampton Station)
	targetLat, targetLon := 51.4158, -0.3713
	point1 := NewPoint(targetLat, targetLon, "point1")
	if !tree.Insert(point1) {
		t.Fatal("Failed to insert point1")
	}
	t.Logf("After insert point1, tree count: %d", tree.Count())

	// Verify we can find it with KNearest
	queryCenter := NewPoint(51.4179, -0.3706, nil) // ~240m away
	queryHalf := queryCenter.HalfPoint(500)        // 500m radius
	queryBox := NewAABB(queryCenter, queryHalf)

	results := tree.KNearest(queryBox, 10, nil)
	found := false
	for _, r := range results {
		if r == point1 {
			found = true
			break
		}
	}
	if !found {
		t.Errorf("After initial insert, KNearest didn't find point1 (got %d results)", len(results))
	} else {
		t.Logf("Initial insert: KNearest found point1")
	}

	// Now simulate an update: Remove old point, insert new one at same location
	if !tree.Remove(point1) {
		t.Fatal("Failed to remove point1")
	}
	t.Logf("After remove, tree count: %d", tree.Count())

	// Insert new point at same coordinates
	point2 := NewPoint(targetLat, targetLon, "point2")
	if !tree.Insert(point2) {
		t.Fatal("Failed to insert point2")
	}
	t.Logf("After insert point2, tree count: %d", tree.Count())

	// Can we find it with DebugFind?
	if !tree.DebugFind(targetLat, targetLon) {
		t.Error("DebugFind can't find the point after Remove+Insert!")
	} else {
		t.Logf("DebugFind found the point after Remove+Insert")
	}

	// Can we find it with KNearest?
	results2 := tree.KNearest(queryBox, 10, nil)
	found2 := false
	for _, r := range results2 {
		if r == point2 {
			found2 = true
			break
		}
	}
	if !found2 {
		t.Errorf("After Remove+Insert, KNearest didn't find point2 (got %d results)", len(results2))

		// Debug: what DID we find?
		t.Logf("Results found:")
		for i, r := range results2 {
			x, y := r.Coordinates()
			t.Logf("  %d: (%.4f, %.4f) data=%v", i, x, y, r.Data())
		}
	} else {
		t.Logf("After Remove+Insert: KNearest found point2")
	}

	// Do multiple Remove+Insert cycles
	currentPoint := point2
	for cycle := 0; cycle < 10; cycle++ {
		if !tree.Remove(currentPoint) {
			t.Fatalf("Cycle %d: Failed to remove", cycle)
		}
		newPoint := NewPoint(targetLat, targetLon, cycle)
		if !tree.Insert(newPoint) {
			t.Fatalf("Cycle %d: Failed to insert", cycle)
		}
		currentPoint = newPoint

		// Check if findable
		results := tree.KNearest(queryBox, 10, nil)
		found := false
		for _, r := range results {
			if r == currentPoint {
				found = true
				break
			}
		}
		if !found {
			t.Errorf("Cycle %d: KNearest can't find point (tree count: %d, DebugFind: %v)",
				cycle, tree.Count(), tree.DebugFind(targetLat, targetLon))
		}
	}
}
