package quadtree

import (
	"math"
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

func New(boundary *AABB, depth int, parent *QuadTree) *QuadTree {
	return &QuadTree{
		boundary: boundary,
		depth:    depth,
		parent:   parent,
	}
}

func NewAABB(center, half *Point) *AABB {
	return &AABB{center, half}
}

func NewPoint(x, y float64, data interface{}) *Point {
	return &Point{x, y, data}
}

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

func (p *Point) Coordinates() (float64, float64) {
	return p.x, p.y
}

func (p *Point) Data() interface{} {
	return p.data
}

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

	qt.nodes[0] = New(bb, qt.depth, qt)

	bb = &AABB{
		&Point{qt.boundary.center.x + qt.boundary.half.x/2, qt.boundary.center.y + qt.boundary.half.y/2, nil},
		&Point{qt.boundary.half.x / 2, qt.boundary.half.y / 2, nil},
	}

	qt.nodes[1] = New(bb, qt.depth, qt)

	bb = &AABB{
		&Point{qt.boundary.center.x - qt.boundary.half.x/2, qt.boundary.center.y - qt.boundary.half.y/2, nil},
		&Point{qt.boundary.half.x / 2, qt.boundary.half.y / 2, nil},
	}

	qt.nodes[2] = New(bb, qt.depth, qt)

	bb = &AABB{
		&Point{qt.boundary.center.x + qt.boundary.half.x/2, qt.boundary.center.y - qt.boundary.half.y/2, nil},
		&Point{qt.boundary.half.x / 2, qt.boundary.half.y / 2, nil},
	}

	qt.nodes[3] = New(bb, qt.depth, qt)

	for _, p := range qt.points {
		for _, node := range qt.nodes {
			if node.Insert(p) {
				break
			}
		}
	}

	qt.points = nil
}

func (qt *QuadTree) knearest(a *AABB, i int, v map[*QuadTree]bool, fn filter) []*Point {
	var results []*Point

	if !qt.boundary.Intersect(a) {
		return results
	}

	if _, ok := v[qt]; ok {
		return results
	} else {
		v[qt] = true
	}

	for _, p := range qt.points {
		if a.ContainsPoint(p) {
			results = append(results, p)
		}

		if len(results) >= i {
			return results[:i]
		}
	}

	if qt.nodes[0] != nil {
		for _, node := range qt.nodes {
			results = append(results, node.knearest(a, i, v, fn)...)

			if len(results) >= i {
				return results[:i]
			}
		}
		if len(results) >= i {
			results = results[:i]
		}
	}

	if qt.parent == nil {
		return results
	}

	return qt.parent.knearest(a, i, v, fn)
}

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

func (qt *QuadTree) KNearest(a *AABB, i int, fn filter) []*Point {
	var results []*Point

	if !qt.boundary.Intersect(a) {
		return results
	}

	// hit the leaf
	if qt.nodes[0] == nil {
		v := make(map[*QuadTree]bool)
		results = append(results, qt.knearest(a, i, v, fn)...)

		if len(results) >= i {
			results = results[:i]
		}

		return results
	}

	for _, node := range qt.nodes {
		results = append(results, node.KNearest(a, i, fn)...)

		if len(results) >= i {
			return results[:i]
		}
	}

	if len(results) >= i {
		results = results[:i]
	}

	return results
}

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
