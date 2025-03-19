/*
 * @Author: bz2021
 * @Date: 2025-02-21 11:05:57
 * @Description:
 */
package rrt

import (
	"image/color"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
)

func addObstacle(p *plot.Plot, obstacles []*Obstacle, cl color.Color) {
	for _, obs := range obstacles {
		pts := plotter.XYs{
			{X: obs.X, Y: obs.Y},
			{X: obs.X + obs.Width, Y: obs.Y},
			{X: obs.X + obs.Width, Y: obs.Y + obs.Height},
			{X: obs.X, Y: obs.Y + obs.Height},
			{X: obs.X, Y: obs.Y},
		}
		pl, _ := plotter.NewPolygon(pts)
		pl.Color = cl
		p.Add(pl)
	}
}

// PlotRRT plots the RRT tree and the final path.
func PlotRRT(r *RRT, filename string) error {
	p := plot.New()

	// Plot obstacles
	addObstacle(p, r.Obstacles, color.Black)

	// Plot the RRT tree
	for _, edge := range r.PathE {
		line := plotter.XYs{
			{X: edge[0].X, Y: edge[0].Y},
			{X: edge[1].X, Y: edge[1].Y},
		}
		pl, _ := plotter.NewLine(line)
		pl.Color = color.Black
		p.Add(pl)
	}

	// Plot the final path
	var finalPath plotter.XYs
	for _, p := range r.Path {
		finalPath = append(finalPath, plotter.XY{
			X: p.X,
			Y: p.Y,
		})
	}
	pl, _ := plotter.NewLine(finalPath)
	pl.Color = color.RGBA{255, 0, 0, 255} // RED
	pl.LineStyle.Width = vg.Points(3)
	p.Add(pl)

	// Save the plot to a file
	if err := p.Save(10*vg.Inch, 10*vg.Inch, filename); err != nil {
		return err
	}

	return nil
}
