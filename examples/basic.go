package main

import (
	"fmt"
	"log"

	"github.com/asim/quadtree"
)

func main() {
	fmt.Println("QuadTree Basic Example")
	fmt.Println("======================\n")

	// Create a quadtree fitting the world geographic bounds (from [-90,-180] to [90,180])
	centerPoint := quadtree.NewPoint(0.0, 0.0, nil)
	halfPoint := quadtree.NewPoint(90.0, 180.0, nil)
	boundingBox := quadtree.NewAABB(centerPoint, halfPoint)

	qtree := quadtree.New(boundingBox, 0, nil)
	fmt.Println("Created quadtree with world geographic bounds")

	// Insert some major cities
	cities := []struct {
		name string
		lat  float64
		lng  float64
	}{
		{"Berlin", 52.5200, 13.4050},
		{"Paris", 48.8566, 2.3522},
		{"London", 51.5074, -0.1278},
		{"New York", 40.7128, -74.0060},
		{"Tokyo", 35.6762, 139.6503},
		{"Sydney", -33.8688, 151.2093},
		{"São Paulo", -23.5505, -46.6333},
		{"Moscow", 55.7558, 37.6173},
		{"Cairo", 30.0444, 31.2357},
		{"Mumbai", 19.0760, 72.8777},
	}

	fmt.Println("\nInserting cities into the quadtree:")
	for _, city := range cities {
		point := quadtree.NewPoint(city.lat, city.lng, city.name)
		if !qtree.Insert(point) {
			log.Fatalf("Failed to insert %s", city.name)
		}
		fmt.Printf("  ✓ Inserted %s (%.4f, %.4f)\n", city.name, city.lat, city.lng)
	}

	// Search for cities in a specific region (around Europe)
	fmt.Println("\n--- Search Example ---")
	searchCenter := quadtree.NewPoint(50.0, 10.0, nil)
	searchHalf := quadtree.NewPoint(10.0, 20.0, nil)
	searchBounds := quadtree.NewAABB(searchCenter, searchHalf)

	fmt.Printf("Searching for cities around Europe (center: %.1f, %.1f)\n", 50.0, 10.0)
	results := qtree.Search(searchBounds)
	fmt.Printf("Found %d cities:\n", len(results))
	for _, point := range results {
		lat, lng := point.Coordinates()
		fmt.Printf("  - %s (%.4f, %.4f)\n", point.Data().(string), lat, lng)
	}

	// Find k-nearest cities to a specific location
	fmt.Println("\n--- K-Nearest Search Example ---")
	queryLat, queryLng := 50.0, 10.0
	queryCenter := quadtree.NewPoint(queryLat, queryLng, nil)
	// Create a search radius of approximately 10,000 km
	// HalfPoint method calculates geographic distance using WGS-84 ellipsoid
	queryDistance := 10000000.0 // meters
	queryBounds := quadtree.NewAABB(queryCenter, queryCenter.HalfPoint(queryDistance))

	maxPoints := 5
	fmt.Printf("Finding %d nearest cities to location (%.1f, %.1f):\n", maxPoints, queryLat, queryLng)
	nearestPoints := qtree.KNearest(queryBounds, maxPoints, nil)
	for i, point := range nearestPoints {
		lat, lng := point.Coordinates()
		fmt.Printf("  %d. %s (%.4f, %.4f)\n", i+1, point.Data().(string), lat, lng)
	}

	// Demonstrate filtering with K-nearest search
	fmt.Println("\n--- Filtered K-Nearest Search Example ---")
	// Filter to only find cities with names starting with 'M' or 'P'
	filterFunc := func(p *quadtree.Point) bool {
		name := p.Data().(string)
		return len(name) > 0 && (name[0] == 'M' || name[0] == 'P')
	}

	fmt.Println("Finding nearest cities starting with 'M' or 'P':")
	filteredPoints := qtree.KNearest(queryBounds, 3, filterFunc)
	for i, point := range filteredPoints {
		lat, lng := point.Coordinates()
		fmt.Printf("  %d. %s (%.4f, %.4f)\n", i+1, point.Data().(string), lat, lng)
	}

	// Demonstrate update functionality
	fmt.Println("\n--- Update Example ---")
	// Note: In a real scenario, you'd keep a reference to the point when inserting
	// For demonstration, we'll insert a new point and update it
	testPoint := quadtree.NewPoint(52.5200, 13.4050, "Berlin-Test")
	qtree.Insert(testPoint)
	newBerlinLocation := quadtree.NewPoint(52.5300, 13.4150, nil)
	fmt.Printf("Updating Berlin-Test from (52.5200, 13.4050) to (52.5300, 13.4150)\n")
	if qtree.Update(testPoint, newBerlinLocation) {
		lat, lng := testPoint.Coordinates()
		fmt.Printf("  ✓ Successfully updated position to (%.4f, %.4f)\n", lat, lng)
	}

	// Demonstrate removal
	fmt.Println("\n--- Remove Example ---")
	if qtree.Remove(testPoint) {
		fmt.Println("  ✓ Successfully removed Berlin-Test")
	}

	fmt.Println("\nExample completed successfully!")
}
