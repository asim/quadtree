QuadTree
========

Golang implementation of the quadtree algorithm. Includes removal, update and knearest search.

[Godoc](https://pkg.go.dev/github.com/asim/quadtree)

## Usage Example

Create a quadtree fitting the world geographic bounds (from [-90,-180] to [90,180])

```go
centerPoint := quadtree.NewPoint(0.0, 0.0, nil)
halfPoint := quadtree.NewPoint(90.0, 180.0, nil)
boundingBox := quadtree.NewAABB(centerPoint, halfPoint)

qtree := quadtree.New(boundingBox, 0, nil)
```

Insert a point into the tree

```go
point := quadtree.NewPoint(52.5200, 13.4050, "Berlin")
if !qtree.Insert(point) {
  log.Fatal("Failed to insert the point")
}
```

Find the k-nearest points (results are sorted by distance to the query center, and duplicates are removed):

```go
center := quadtree.NewPoint(lat, lng, nil)
distance := 10000 /* Distance to the center point in meters */
bounds := quadtree.NewAABB(center, center.HalfPoint(distance))

maxPoints := 10
for _, point := range qtree.KNearest(bounds, maxPoints, nil) {
  log.Printf("Found point: %s\n", point.Data().(string))
}
```

## HTTP Server

A web-based HTTP server with REST API and interactive UI is available for managing and visualizing points.

![QuadTree Viewer](https://github.com/user-attachments/assets/f7f3f3a6-ff29-4a7e-86ed-9dccd579e067)

```bash
cd cmd/server
go run main.go
```

Visit `http://localhost:8080` in your browser for an interactive grid visualization with:
- REST API endpoints for CRUD operations
- Visual point management with mouse/keyboard navigation
- Regional search functionality
- Real-time updates
- Responsive mobile-first design

See [cmd/server/README.md](./cmd/server/README.md) for full API documentation and usage.

## Examples

For complete, runnable examples, see the [examples directory](./examples):

- **[simple.go](./examples/simple.go)** - A minimal example demonstrating basic quadtree operations (insert, search, k-nearest)
- **[basic.go](./examples/basic.go)** - A comprehensive example with real-world city coordinates, filtering, updates, and removals

Run any example with:

```bash
cd examples
go run simple.go
```

## Notes
- `KNearest` returns up to `k` points, sorted by Euclidean distance to the query center.
- Duplicate points are removed from the result.
- The distance metric is Euclidean (straight-line). For geospatial data, you may want to adapt this to use Haversine or another metric if needed.
