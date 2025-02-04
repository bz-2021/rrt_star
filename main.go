package main

import (
	"fmt"
	"image/color"
	"math"
	"math/rand"
	"time"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
)

// 全局变量
var (
	Ll             []float64
	Tt             []float64
	Node           []int
	All_Iterations []int
	goalPose       = [2]float64{999, 999}
	step           = 20.0
	kp             = 1.0
	p0             = 250.0
	krep           = 1.0
	bias           = 1.5 * step
	xMax           = 1000.0
	yMax           = 1000.0
	obstacle1      = [4]float64{550, 150, 200, 200}
	obstacle2      = [4]float64{600, 550, 200, 200}
	obstacle3      = [4]float64{200, 200, 200, 300}
	obstacle11     [4]float64
	obstacle22     [4]float64
	obstacle33     [4]float64
	pathV          [][3]float64
)

func main() {
	rand.Seed(time.Now().UnixNano())

	// 初始化障碍物
	or := 10.0
	obstacle11 = [4]float64{obstacle1[0] - or, obstacle1[1] - or, obstacle1[2] + 2*or, obstacle1[3] + 2*or}
	obstacle22 = [4]float64{obstacle2[0] - or, obstacle2[1] - or, obstacle2[2] + 2*or, obstacle2[3] + 2*or}
	obstacle33 = [4]float64{obstacle3[0] - or, obstacle3[1] - or, obstacle3[2] + 2*or, obstacle3[3] + 2*or}

	// 主循环
	for j := 0; j < 2; j++ {
		// 创建绘图
		p := plot.New()
		p.Title.Text = "RRT Path Planning"
		p.X.Label.Text = "X"
		p.Y.Label.Text = "Y"
		p.X.Min = 0
		p.X.Max = xMax
		p.Y.Min = 0
		p.Y.Max = yMax

		// 绘制障碍物
		drawObstacle(p, obstacle1, "darkgray")
		drawObstacle(p, obstacle2, "darkgray")
		drawObstacle(p, obstacle3, "darkgray")

		startPose := [2]float64{0, 0}
		pathV = [][3]float64{{startPose[0], startPose[1], -1}}
		counter := 0
		numNodes := 5000
		pVal := 0.3 // 避免与绘图对象 p 冲突

		startTime := time.Now()

		for i := 0; i < numNodes; i++ {
			var qRand [2]float64
			if rand.Float64() <= pVal {
				qRand = goalPose
			} else {
				qRand = [2]float64{rand.Float64() * xMax, rand.Float64() * yMax}
			}

			np, I := nearestPoint(qRand)
			newPoint := apfNewPoint(np, qRand)

			if noCollision(newPoint, np, obstacle11) && noCollision(newPoint, np, obstacle22) && noCollision(newPoint, np, obstacle33) {
				// 绘制路径
				line := plotter.XYs{{X: np[0], Y: np[1]}, {X: newPoint[0], Y: newPoint[1]}}
				pl, _ := plotter.NewLine(line)
				pl.Color = color.Black
				p.Add(pl)

				counter++
				pathV = append(pathV, [3]float64{newPoint[0], newPoint[1], float64(I)})
			}

			if math.Sqrt(math.Pow(newPoint[0]-goalPose[0], 2)+math.Pow(newPoint[1]-goalPose[1], 2)) <= bias {
				break
			}
		}

		// 绘制最终路径
		pose1 := pathV[len(pathV)-1]
		var path [][2]float64
		for pose1[2] > 0 {
			path = append([][2]float64{{pose1[0], pose1[1]}}, path...)
			pose1 = pathV[int(pose1[2])]
		}
		path = append([][2]float64{{pathV[0][0], pathV[0][1]}}, path...)

		// 绘制最终路径
		var finalPath plotter.XYs
		for _, point := range path {
			finalPath = append(finalPath, plotter.XY{X: point[0], Y: point[1]})
		}
		pl, _ := plotter.NewLine(finalPath)
		pl.Color = color.RGBA{255, 0, 0, 255}
		p.Add(pl)

		// 保存图像
		if err := p.Save(10*vg.Inch, 10*vg.Inch, fmt.Sprintf("rrt_path_%d.png", j)); err != nil {
			panic(err)
		}

		// 计算路径长度
		L := 0.0
		for i := 0; i < len(path)-1; i++ {
			L += math.Sqrt(math.Pow(path[i][0]-path[i+1][0], 2) + math.Pow(path[i][1]-path[i+1][1], 2))
		}

		Ll = append(Ll, L)
		Tt = append(Tt, time.Since(startTime).Seconds())
		Node = append(Node, len(path))
		All_Iterations = append(All_Iterations, numNodes)
	}

	fmt.Println("Average original path length:", mean(Ll))
	fmt.Println("Average experiment time:", mean(Tt))
	fmt.Println("Average number of experimental path nodes:", meanInt(Node))
	fmt.Println("Average number of experiment iterations:", meanInt(All_Iterations))
}

// 绘制障碍物
func drawObstacle(p *plot.Plot, obstacle [4]float64, cl string) {
	pts := plotter.XYs{
		{X: obstacle[0], Y: obstacle[1]},
		{X: obstacle[0] + obstacle[2], Y: obstacle[1]},
		{X: obstacle[0] + obstacle[2], Y: obstacle[1] + obstacle[3]},
		{X: obstacle[0], Y: obstacle[1] + obstacle[3]},
		{X: obstacle[0], Y: obstacle[1]},
	}
	pl, _ := plotter.NewPolygon(pts)
	pl.Color = color.Black
	p.Add(pl)
}

func mean(arr []float64) float64 {
	sum := 0.0
	for _, v := range arr {
		sum += v
	}
	return sum / float64(len(arr))
}

func meanInt(arr []int) float64 {
	sum := 0
	for _, v := range arr {
		sum += v
	}
	return float64(sum) / float64(len(arr))
}