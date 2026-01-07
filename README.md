# QuadTree

A fast, production-ready spatial index for Go. Insert, remove, update, and search millions of points with k-nearest neighbor queries.

Used for location-based services, game development, spatial analytics, and anywhere you need efficient 2D point lookups.

[![GoDoc](https://pkg.go.dev/badge/github.com/asim/quadtree)](https://pkg.go.dev/github.com/asim/quadtree)
[![Go Report Card](https://goreportcard.com/badge/github.com/asim/quadtree)](https://goreportcard.com/report/github.com/asim/quadtree)

## Features

- **Fast spatial queries** - K-nearest neighbor search with distance sorting
- **Full CRUD** - Insert, update, remove, and query points
- **Persistence** - Built-in file storage with JSON or custom backends
- **HTTP API** - Ready-to-use REST endpoints for your application
- **Interactive UI** - Embeddable visualization for debugging and management
- **Zero dependencies** - Pure Go, no external libraries
- **Battle-tested** - Used in production for location services

## Install

```bash
go get github.com/asim/quadtree
```

## Quick Start

```go
package main

import (
    "fmt"
    "github.com/asim/quadtree"
)

func main() {
    // Create a quadtree covering the world
    center := quadtree.NewPoint(0, 0, nil)
    half := quadtree.NewPoint(90, 180, nil)
    bounds := quadtree.NewAABB(center, half)
    tree := quadtree.New(bounds, 0, nil)

    // Insert some cities
    tree.Insert(quadtree.NewPoint(51.5074, -0.1278, "London"))
    tree.Insert(quadtree.NewPoint(48.8566, 2.3522, "Paris"))
    tree.Insert(quadtree.NewPoint(52.5200, 13.4050, "Berlin"))

    // Find 2 nearest cities to a point
    query := quadtree.NewPoint(50.0, 5.0, nil)
    searchBounds := quadtree.NewAABB(query, query.HalfPoint(500000))
    
    for _, p := range tree.KNearest(searchBounds, 2, nil) {
        fmt.Printf("Found: %s\n", p.Data().(string))
    }
}
```

## HTTP API

Embed a REST API in your application:

```go
import "github.com/asim/quadtree"

// Create tree with persistent storage
tree := quadtree.New(bounds, 0, nil)
store, _ := quadtree.NewFileStore("points.json")
handler := quadtree.NewHTTPHandler(tree, store)

http.Handle("/api/", http.StripPrefix("/api", handler))
http.ListenAndServe(":8080", nil)
```

**Endpoints:**

| Method | Path | Description |
|--------|------|-------------|
| GET | /points | List all points |
| POST | /points | Add a point |
| GET | /points/{id} | Get a point |
| DELETE | /points/{id} | Delete a point |
| POST | /search | Search within radius |

## Interactive UI

![QuadTree Viewer](https://github.com/user-attachments/assets/f7f3f3a6-ff29-4a7e-86ed-9dccd579e067)

A built-in visualization UI for debugging and point management:

```go
import "github.com/asim/quadtree/ui"

// Add UI to your server
http.Handle("/", ui.Handler(ui.DefaultConfig()))
```

Features:
- Pan/zoom with mouse, touch, keyboard
- Real-time point visualization
- CRUD operations via UI
- Mobile-responsive
- Customizable appearance

## Standalone Server

Run the included server for quick testing:

```bash
cd cmd/server
go run main.go
# Visit http://localhost:8080
```

## Use Cases

- **Location services** - Find nearby restaurants, stores, users
- **Game development** - Spatial collision detection, entity lookup
- **Mapping** - Efficient point-of-interest queries
- **Analytics** - Geospatial data clustering and search
- **IoT** - Device location tracking and proximity alerts

## Performance

Quadtree provides O(log n) average case for insertions and queries, making it suitable for datasets from thousands to millions of points.

## Examples

See the [examples directory](./examples) for complete, runnable code:

```bash
cd examples
go run simple.go   # Basic operations
go run basic.go    # Real-world cities demo
```

## Enterprise Support

Using QuadTree in production? Need custom features, priority support, or consulting?

**[Create an issue](https://github.com/asim/quadtree/issues/new)** or contact for enterprise inquiries.

## License

[Apache 2.0](LICENSE)

---

Built by [Asim Aslam](https://github.com/asim) • Part of [Mu.XYZ](https://mu.xyz)
