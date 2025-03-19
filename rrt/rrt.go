/*
 * @Author: bz2021
 * @Date: 2025-02-21 10:13:21
 * @Description:
 */
package rrt

import (
	"math"
)

// Point represents a 2D point.
type Point struct {
	X, Y float64
}

// RRT represents the RRT algorithm.
type RRT struct {
	Start          Point
	Goal           Point
	Step           float64
	Bias           float64
	NumNodes       int
	XMax, YMax     float64
	Obstacles      []*Obstacle
	InfluenceRange float64
	PathV          []Point    // 树中的节点
	Parent         []int      // 节点的索引，存储当前节点的父节点
	PathE          [][2]Point // 树中的边
	Path           []Point    // 最终的路径
}

// NewRRT creates a new RRT instance.
func NewRRT(start, goal Point, step, bias float64, numNodes int, xMax, yMax float64, obstacles []*Obstacle, influenceRange float64) *RRT {
	return &RRT{
		Start:          start,
		Goal:           goal,
		Step:           step,
		Bias:           bias,
		NumNodes:       numNodes,
		XMax:           xMax,
		YMax:           yMax,
		Obstacles:      obstacles,
		InfluenceRange: influenceRange,
		PathV:          []Point{start},
		Parent:         []int{-1},
		PathE:          [][2]Point{},
		Path:           []Point{},
	}
}

// NearestPoint finds the nearest point in the tree to a given point.
func (r *RRT) NearestPoint(randomPoint Point) (Point, int) {
	minDist := math.MaxFloat64
	var nearestPoint Point
	var index int

	for i, p := range r.PathV {
		dist := r.EuclideanDistance(p, randomPoint)
		if dist < minDist {
			minDist = dist
			nearestPoint = p
			index = i
		}
	}

	return nearestPoint, index
}

// NewPoint generates a new point towards the random point.
func (r *RRT) NewPoint(nearestPoint, randomPoint Point) Point {
	dir := r.Direction(nearestPoint, randomPoint)
	len := r.EuclideanDistance(nearestPoint, randomPoint)

	if len < r.Step {
		return randomPoint
	}

	return Point{
		X: nearestPoint.X + dir.X*r.Step,
		Y: nearestPoint.Y + dir.Y*r.Step,
	}
}

// NoCollision checks if the path between two points is collision-free.
func (r *RRT) NoCollision(p1, p2 Point) bool {
	for _, obs := range r.Obstacles {
		if r.CheckLineIntersection(p1, p2, obs) {
			return false
		}
	}
	return true
}

// CheckLineIntersection checks if a line intersects with an obstacle.
func (r *RRT) CheckLineIntersection(p1, p2 Point, obs *Obstacle) bool {
	// Implement line-rectangle intersection logic here
	// This is a simplified version, you may need to implement a more accurate collision detection
	x1, y1, x2, y2 := p1.X, p1.Y, p2.X, p2.Y
	ox, oy, ow, oh := obs.GetBounds(r.InfluenceRange)

	// Check if the line intersects with the obstacle's bounds
	// This is a placeholder, you should implement a proper line-rectangle intersection test
	return x1 >= ox && x1 <= ox+ow && y1 >= oy && y1 <= oy+oh ||
		x2 >= ox && x2 <= ox+ow && y2 >= oy && y2 <= oy+oh
}

// EuclideanDistance calculates the Euclidean distance between two points.
func (r *RRT) EuclideanDistance(p1, p2 Point) float64 {
	return math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2))
}

// Direction calculates the direction vector from p1 to p2.
func (r *RRT) Direction(p1, p2 Point) Point {
	dx := p2.X - p1.X
	dy := p2.Y - p1.Y
	len := r.EuclideanDistance(p1, p2)
	return Point{X: dx / len, Y: dy / len}
}

// ExtractPath extracts the final path from the goal to the start.
func (r *RRT) ExtractPath(goalIndex int) {
	r.Path = []Point{}
	currentIndex := goalIndex

	for currentIndex != -1 {
		r.Path = append(r.Path, r.PathV[currentIndex])
		currentIndex = r.Parent[currentIndex]
	}

	// Reverse the path to get it from start to goal
	for i, j := 0, len(r.Path)-1; i < j; i, j = i+1, j-1 {
		r.Path[i], r.Path[j] = r.Path[j], r.Path[i]
	}
}