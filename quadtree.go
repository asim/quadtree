package quadtree

import (
	"math"
	"sort"
)

var (
	Capacity = 8
	MaxDepth = 6
)

type AABB struct {
	center *Point
	half   *Point
}

type Point struct {
	x    float64
	y    float64
	data interface{}
}

type QuadTree struct {
	boundary *AABB
	depth    int
	points   []*Point
	parent   *QuadTree
	nodes    [4]*QuadTree
}

type filter func(*Point) bool

func deg2Rad(deg float64) float64 {
	return deg * (math.Pi / 180)
}

func rad2Deg(rad float64) float64 {
	return (180.0 * rad) / math.Pi
}

func boundaryPoint(x *Point, m float64) *Point {
	x2 := deg2Rad(x.x)
	y2 := deg2Rad(x.y)

	// Radius of Earth at given latitude
	radius := earthRadius(x2)
	// Radius of the parallel at given latitude
	pradius := radius * math.Cos(x2)

	xMax := x2 + m/radius
	yMax := y2 + m/pradius

	return &Point{rad2Deg(xMax), rad2Deg(yMax), nil}
}

// Earth radius at a given latitude, according to the WGS-84 ellipsoid [m]
func earthRadius(x float64) float64 {
	masm := 6378137.0 // Major semiaxis [m]
	mism := 6356752.3 // Minor semiaxis [m]

	an := masm * masm * math.Cos(x)
	bn := mism * mism * math.Sin(x)
	ad := masm * math.Cos(x)
	bd := mism * math.Sin(x)
	return math.Sqrt((an*an + bn*bn) / (ad*ad + bd*bd))
}

// New creates a new *QuadTree. It requires a boundary defining the center
// and half points, depth at which the QuadTree resides and parent node.
// Depth of 0 and parent as nil implies the root node.
func New(boundary *AABB, depth int, parent *QuadTree) *QuadTree {
	return &QuadTree{
		boundary: boundary,
		depth:    depth,
		parent:   parent,
	}
}

// NewAABB creates an axis aligned bounding box. It takes the center and half
// point.
func NewAABB(center, half *Point) *AABB {
	return &AABB{center, half}
}

// NewPoint generates a new *Point struct.
func NewPoint(x, y float64, data interface{}) *Point {
	return &Point{x, y, data}
}

// ContainsPoint checks whether the point provided resides within the axis
// aligned bounding box.
func (a *AABB) ContainsPoint(p *Point) bool {
	if p.x < a.center.x-a.half.x {
		return false
	}
	if p.y < a.center.y-a.half.y {
		return false
	}
	if p.x > a.center.x+a.half.x {
		return false
	}
	if p.y > a.center.y+a.half.y {
		return false
	}

	return true
}

// Intersect checks whether two axis aligned bounding boxes overlap.
func (a *AABB) Intersect(b *AABB) bool {
	if b.center.x+b.half.x < a.center.x-a.half.x {
		return false
	}
	if b.center.y+b.half.y < a.center.y-a.half.y {
		return false
	}
	if b.center.x-b.half.x > a.center.x+a.half.x {
		return false
	}
	if b.center.y-b.half.y > a.center.y+a.half.y {
		return false
	}

	return true
}

// Coordinates return the x and y coordinates of a point.
func (p *Point) Coordinates() (float64, float64) {
	return p.x, p.y
}

// Data returns the data stored within a point.
func (p *Point) Data() interface{} {
	return p.data
}

// HalfPoint is a convenience function for generating the half point
// required to created an axis aligned bounding box. It takes an
// argument of metres as float64.
func (p *Point) HalfPoint(m float64) *Point {
	p2 := boundaryPoint(p, m)
	return &Point{p2.x - p.x, p2.y - p.y, nil}
}

func (qt *QuadTree) divide() {
	if qt.nodes[0] != nil {
		return
	}

	bb := &AABB{
		&Point{qt.boundary.center.x - qt.boundary.half.x/2, qt.boundary.center.y + qt.boundary.half.y/2, nil},
		&Point{qt.boundary.half.x / 2, qt.boundary.half.y / 2, nil},
	}

	qt.nodes[0] = New(bb, qt.depth+1, qt)

	bb = &AABB{
		&Point{qt.boundary.center.x + qt.boundary.half.x/2, qt.boundary.center.y + qt.boundary.half.y/2, nil},
		&Point{qt.boundary.half.x / 2, qt.boundary.half.y / 2, nil},
	}

	qt.nodes[1] = New(bb, qt.depth+1, qt)

	bb = &AABB{
		&Point{qt.boundary.center.x - qt.boundary.half.x/2, qt.boundary.center.y - qt.boundary.half.y/2, nil},
		&Point{qt.boundary.half.x / 2, qt.boundary.half.y / 2, nil},
	}

	qt.nodes[2] = New(bb, qt.depth+1, qt)

	bb = &AABB{
		&Point{qt.boundary.center.x + qt.boundary.half.x/2, qt.boundary.center.y - qt.boundary.half.y/2, nil},
		&Point{qt.boundary.half.x / 2, qt.boundary.half.y / 2, nil},
	}

	qt.nodes[3] = New(bb, qt.depth+1, qt)

	for _, p := range qt.points {
		for _, node := range qt.nodes {
			if node.Insert(p) {
				break
			}
		}
	}

	qt.points = nil
}

func distance(a, b *Point) float64 {
	dx := a.x - b.x
	dy := a.y - b.y
	return math.Sqrt(dx*dx + dy*dy)
}

func (qt *QuadTree) knearest(a *AABB, i int, v map[*QuadTree]bool, fn filter) []*Point {
	var results []*Point

	if _, ok := v[qt]; ok {
		return results
	} else {
		v[qt] = true
	}

	if !qt.boundary.Intersect(a) {
		return results
	}

	for _, p := range qt.points {
		if a.ContainsPoint(p) {
			if fn == nil || fn(p) {
				results = append(results, p)
			}
		}
	}

	if qt.nodes[0] != nil {
		for _, node := range qt.nodes {
			results = append(results, node.knearest(a, i, v, fn)...)
		}
	}

	if qt.parent != nil {
		results = append(results, qt.parent.knearest(a, i, v, fn)...)
	}

	// Sort by distance to the center of the query AABB
	center := a.center
	sort.Slice(results, func(i, j int) bool {
		return distance(results[i], center) < distance(results[j], center)
	})

	if len(results) > i {
		results = results[:i]
	}
	return results
}

// Insert will attempt to insert the point into the QuadTree. It will
// recursively search until it finds the leaf node. If the leaf node
// is at capacity then it will try split the node. If the tree is at
// max depth then point will be stored in the leaf.
func (qt *QuadTree) Insert(p *Point) bool {
	if !qt.boundary.ContainsPoint(p) {
		return false
	}

	if qt.nodes[0] == nil {
		if len(qt.points) < Capacity {
			qt.points = append(qt.points, p)
			return true
		}

		if qt.depth < MaxDepth {
			qt.divide()
		} else {
			qt.points = append(qt.points, p)
			return true
		}
	}

	for _, node := range qt.nodes {
		if node.Insert(p) {
			return true
		}
	}

	return false
}

// KNearest returns the k nearest points within the QuadTree that fall within
// the bounds of the axis aligned bounding box. A filter function can be used
// which is evaluated against each point. The search begins at the leaf and
// recurses towards the parent until k nearest have been found or root node is
// hit.
func (qt *QuadTree) kNearestRoot(a *AABB, i int, v map[*QuadTree]bool, fn filter) []*Point {
	var results []*Point

	if !qt.boundary.Intersect(a) {
		return results
	}

	// hit the leaf
	if qt.nodes[0] == nil {
		results = append(results, qt.knearest(a, i, v, fn)...)

		if len(results) >= i {
			results = results[:i]
		}

		return results
	}

	for _, node := range qt.nodes {
		results = append(results, node.kNearestRoot(a, i, v, fn)...)

		if len(results) >= i {
			return results[:i]
		}
	}

	if len(results) >= i {
		results = results[:i]
	}

	return results
}

func (qt *QuadTree) KNearest(a *AABB, i int, fn filter) []*Point {
	v := make(map[*QuadTree]bool)
	return qt.kNearestRoot(a, i, v, fn)
}

// Remove attemps to remove a point from the QuadTree. It will recurse until
// the leaf node is found and then try to remove the point.
func (qt *QuadTree) Remove(p *Point) bool {
	if !qt.boundary.ContainsPoint(p) {
		return false
	}

	if qt.nodes[0] == nil {
		for i, ep := range qt.points {
			if ep != p {
				continue
			}

			// remove point
			if last := len(qt.points) - 1; i == last {
				qt.points = qt.points[:last]
			} else {
				qt.points[i] = qt.points[last]
				qt.points = qt.points[:last]
			}
			return true
		}

		return false
	}

	for _, node := range qt.nodes {
		if node.Remove(p) {
			return true
		}
	}

	return false
}

// RInsert is used in conjuction with Update to try reveser insert a point.
func (qt *QuadTree) RInsert(p *Point) bool {
	// Try insert down the tree
	if qt.Insert(p) {
		return true
	}

	// hit root node
	if qt.parent == nil {
		return false
	}

	// try rinsert parent
	return qt.parent.RInsert(p)
}

// Search will return all the points within the given axis aligned bounding
// box. It recursively searches downward through the tree.
func (qt *QuadTree) Search(a *AABB) []*Point {
	var results []*Point

	if !qt.boundary.Intersect(a) {
		return results
	}

	for _, p := range qt.points {
		if a.ContainsPoint(p) {
			results = append(results, p)
		}
	}

	if qt.nodes[0] == nil {
		return results
	}

	for _, node := range qt.nodes {
		results = append(results, node.Search(a)...)
	}

	return results
}

// Update will update the location of a point within the tree. It is
// optimised to attempt reinsertion within the same node and recurse
// back up the tree until it finds a suitable node.
func (qt *QuadTree) Update(p *Point, np *Point) bool {
	if !qt.boundary.ContainsPoint(p) {
		return false
	}

	// At the leaf
	if qt.nodes[0] == nil {
		for i, ep := range qt.points {
			if ep != p {
				continue
			}

			// set new coords
			p.x = np.x
			p.y = np.y

			// now do we move?
			if qt.boundary.ContainsPoint(np) {
				return true
			}

			// remove from current node
			if last := len(qt.points) - 1; i == last {
				qt.points = qt.points[:last]
			} else {
				qt.points[i] = qt.points[last]
				qt.points = qt.points[:last]
			}

			// well shit now...reinsert
			return qt.RInsert(p)
		}
		return false
	}

	for _, node := range qt.nodes {
		if node.Update(p, np) {
			return true
		}
	}

	return false
}
