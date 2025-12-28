# QuadTree Examples

This directory contains runnable examples demonstrating how to use the quadtree library.

## Running the Examples

Each example is a standalone Go program that can be run directly:

```bash
go run simple.go
```

or

```bash
go run basic.go
```

## Available Examples

### simple.go

A minimal example showing the basic operations:
- Creating a quadtree with a bounding box
- Inserting points
- Searching for points in a region
- Finding K-nearest neighbors

This is the best starting point for understanding the library.

```bash
cd examples
go run simple.go
```

### basic.go

A comprehensive example demonstrating all major features:
- Creating a quadtree for geographic data (world bounds)
- Inserting multiple points (cities)
- Searching for points in a region
- Finding K-nearest neighbors
- Filtering results with a custom function
- Updating point locations
- Removing points

This example uses real-world city coordinates to show practical usage.

```bash
cd examples
go run basic.go
```

## Quick Start

Here's a minimal code snippet to get started:

```go
package main

import (
	"fmt"
	"github.com/asim/quadtree"
)

func main() {
	// Create a quadtree
	center := quadtree.NewPoint(0.0, 0.0, nil)
	half := quadtree.NewPoint(10.0, 10.0, nil)
	bounds := quadtree.NewAABB(center, half)
	tree := quadtree.New(bounds, 0, nil)

	// Insert a point
	point := quadtree.NewPoint(5.0, 5.0, "My Point")
	tree.Insert(point)

	// Search for points
	results := tree.Search(bounds)
	fmt.Printf("Found %d points\n", len(results))
}
```

## Module Setup

The examples use a local replace directive in their `go.mod` to reference the parent quadtree module. This allows you to run the examples directly from this directory without publishing the module.
