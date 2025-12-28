package main

import (
	"fmt"
	"log"

	"github.com/asim/quadtree"
)

func main() {
	fmt.Println("QuadTree Simple Example\n")

	// Step 1: Create a quadtree with a bounding box
	// This creates a tree covering coordinates from (-10, -10) to (10, 10)
	center := quadtree.NewPoint(0.0, 0.0, nil)
	half := quadtree.NewPoint(10.0, 10.0, nil)
	bounds := quadtree.NewAABB(center, half)
	tree := quadtree.New(bounds, 0, nil)
	fmt.Println("Created quadtree with bounds: center (0, 0), half (10, 10)")

	// Step 2: Insert some points
	fmt.Println("\nInserting points:")
	points := []struct {
		x, y float64
		name string
	}{
		{1.0, 2.0, "Point A"},
		{-3.0, 4.0, "Point B"},
		{5.0, -2.0, "Point C"},
		{0.0, 0.0, "Origin"},
		{7.0, 8.0, "Point D"},
	}

	for _, p := range points {
		point := quadtree.NewPoint(p.x, p.y, p.name)
		if tree.Insert(point) {
			fmt.Printf("  ✓ Inserted %s at (%.1f, %.1f)\n", p.name, p.x, p.y)
		} else {
			log.Fatalf("  ✗ Failed to insert %s", p.name)
		}
	}

	// Step 3: Search for points in a region
	fmt.Println("\nSearching for points in region around (0, 0) with radius 5:")
	searchCenter := quadtree.NewPoint(0.0, 0.0, nil)
	searchHalf := quadtree.NewPoint(5.0, 5.0, nil)
	searchBounds := quadtree.NewAABB(searchCenter, searchHalf)

	results := tree.Search(searchBounds)
	fmt.Printf("Found %d points:\n", len(results))
	for _, point := range results {
		x, y := point.Coordinates()
		fmt.Printf("  - %s at (%.1f, %.1f)\n", point.Data().(string), x, y)
	}

	// Step 4: Find K-nearest neighbors
	fmt.Println("\nFinding 3 nearest points to (0, 0):")
	queryBounds := quadtree.NewAABB(center, half)
	nearest := tree.KNearest(queryBounds, 3, nil)

	for i, point := range nearest {
		x, y := point.Coordinates()
		fmt.Printf("  %d. %s at (%.1f, %.1f)\n", i+1, point.Data().(string), x, y)
	}

	fmt.Println("\n✓ Example completed!")
}
