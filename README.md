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

## Notes
- `KNearest` returns up to `k` points, sorted by Euclidean distance to the query center.
- Duplicate points are removed from the result.
- The distance metric is Euclidean (straight-line). For geospatial data, you may want to adapt this to use Haversine or another metric if needed.
