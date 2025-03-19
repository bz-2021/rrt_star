package rrt

// Obstacle represents a rectangular obstacle in the 2D space.
type Obstacle struct {
	X, Y, Width, Height float64
}

// NewObstacle creates a new Obstacle instance.
func NewObstacle(x, y, width, height float64) *Obstacle {
	return &Obstacle{
		X:      x,
		Y:      y,
		Width:  width,
		Height: height,
	}
}

// Contains checks if a point (x, y) is inside the obstacle.
func (o *Obstacle) Contains(x, y float64) bool {
	return x >= o.X && x <= o.X+o.Width && y >= o.Y && y <= o.Y+o.Height
}

// GetBounds returns the bounds of the obstacle with an influence range.
func (o *Obstacle) GetBounds(influenceRange float64) (float64, float64, float64, float64) {
	return o.X - influenceRange, o.Y - influenceRange, o.Width + 2*influenceRange, o.Height + 2*influenceRange
}
