/*
 * @Author: bz2021
 * @Date: 2025-02-21 19:39:27
 * @Description:
 */
package main

import (
	"fmt"
	"math/rand"
	"time"

	"github.com/bz-2021/rrt_star/rrt"
)

func main() {
	rand.Seed(time.Now().UnixNano())

	// Define the start and goal points
	start := rrt.Point{X: 0, Y: 0}
	goal := rrt.Point{X: 999, Y: 999}

	// Define obstacles
	obstacles := []*rrt.Obstacle{
		rrt.NewObstacle(150, 150, 150, 150),   // 左上
		rrt.NewObstacle(600, 200, 100, 100),   // 右上
		rrt.NewObstacle(200, 600, 100, 100),   // 左下
		rrt.NewObstacle(700, 700, 150, 150),   // 右下
		rrt.NewObstacle(400, 400, 200, 200),   // 中心
	}

	// Initialize RRT
	rrtInstance := rrt.NewRRT(start, goal, 20, 20, 5000, 1000, 1000, obstacles, 10)

	// Run RRT algorithm
	var goalIndex int
	for range rrtInstance.NumNodes {
		var randomPoint rrt.Point
		if rand.Float64() <= 0.3 {
			randomPoint = goal
		} else {
			randomPoint = rrt.Point{X: rand.Float64() * rrtInstance.XMax, Y: rand.Float64() * rrtInstance.YMax}
		}

		nearestPoint, nearestIndex := rrtInstance.NearestPoint(randomPoint)
		newPoint := rrtInstance.NewPoint(nearestPoint, randomPoint)

		if rrtInstance.NoCollision(nearestPoint, newPoint) {
			rrtInstance.PathV = append(rrtInstance.PathV, newPoint)
			rrtInstance.Parent = append(rrtInstance.Parent, nearestIndex)
			rrtInstance.PathE = append(rrtInstance.PathE, [2]rrt.Point{nearestPoint, newPoint})

			if rrtInstance.EuclideanDistance(newPoint, goal) <= rrtInstance.Bias {
				goalIndex = len(rrtInstance.PathV) - 1
				break
			}
		}
	}

	// Extract the final path
	if goalIndex != 0 {
		rrtInstance.ExtractPath(goalIndex)
	} else {
		fmt.Println("No path found.")
	}

	// Plot the RRT tree and path
	if err := rrt.PlotRRT(rrtInstance, "rrt_plot.png"); err != nil {
		fmt.Println("Error plotting RRT:", err)
	}

	fmt.Println("RRT algorithm completed and plot saved.")
}
