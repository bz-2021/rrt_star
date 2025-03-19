// package main

// import (
// 	"fmt"
// 	"image/color"
// 	"math"
// 	"math/rand"
// 	"time"

// 	"gonum.org/v1/plot"
// 	"gonum.org/v1/plot/plotter"
// 	"gonum.org/v1/plot/vg"
// )

// // 全局变量
// var (
// 	Ll             []float64              // 存储每次运行的路径长度
// 	Tt             []float64              // 存储每次运行的时间
// 	Node           []int                  // 存储每次运行的路径节点数
// 	All_Iterations []int                  // 存储每次运行的迭代次数
// 	goalPose       = [2]float64{999, 999} // 目标点坐标
// 	step           = 20.0                 // 步长
// 	kp             = 1.0                  // 引力增益系数
// 	p0             = 250.0                // 斥力作用范围
// 	krep           = 1.0                  // 斥力增益系数
// 	bias           = 1.5 * step           // 允许的误差范围
// 	xMax           = 1000.0               // 地图的 X 轴最大值
// 	yMax           = 1000.0               // 地图的 Y 轴最大值
// 	obstacle1      = [4]float64{550, 150, 200, 200} // 障碍物 1 的坐标和大小 [x, y, width, height]
// 	obstacle2      = [4]float64{600, 550, 200, 200} // 障碍物 2 的坐标和大小
// 	obstacle3      = [4]float64{200, 200, 200, 300} // 障碍物 3 的坐标和大小
// 	obstacle11     [4]float64             // 障碍物 1 的扩展范围
// 	obstacle22     [4]float64             // 障碍物 2 的扩展范围
// 	obstacle33     [4]float64             // 障碍物 3 的扩展范围
// 	pathV          [][3]float64           // 存储随机树的节点 [x, y, parentIndex]
// )

// func main() {
// 	rand.Seed(time.Now().UnixNano()) // 初始化随机数种子

// 	// 初始化障碍物的扩展范围
// 	or := 10.0
// 	obstacle11 = [4]float64{obstacle1[0] - or, obstacle1[1] - or, obstacle1[2] + 2*or, obstacle1[3] + 2*or}
// 	obstacle22 = [4]float64{obstacle2[0] - or, obstacle2[1] - or, obstacle2[2] + 2*or, obstacle2[3] + 2*or}
// 	obstacle33 = [4]float64{obstacle3[0] - or, obstacle3[1] - or, obstacle3[2] + 2*or, obstacle3[3] + 2*or}

// 	// 主循环，运行两次实验
// 	for j := 0; j < 2; j++ {
// 		// 创建绘图对象
// 		p := plot.New()
// 		p.Title.Text = "RRT Path Planning"
// 		p.X.Label.Text = "X"
// 		p.Y.Label.Text = "Y"
// 		p.X.Min = 0
// 		p.X.Max = xMax
// 		p.Y.Min = 0
// 		p.Y.Max = yMax

// 		// 绘制障碍物
// 		drawObstacle(p, obstacle1, "darkgray")
// 		drawObstacle(p, obstacle2, "darkgray")
// 		drawObstacle(p, obstacle3, "darkgray")

// 		startPose := [2]float64{0, 0} // 起点坐标
// 		pathV = [][3]float64{{startPose[0], startPose[1], -1}} // 初始化随机树，起点为根节点
// 		counter := 0
// 		numNodes := 5000 // 最大迭代次数
// 		pVal := 0.3      // 目标偏向概率

// 		startTime := time.Now() // 记录开始时间

// 		// RRT 主循环
// 		for i := 0; i < numNodes; i++ {
// 			var qRand [2]float64
// 			if rand.Float64() <= pVal {
// 				qRand = goalPose // 以概率 pVal 选择目标点
// 			} else {
// 				qRand = [2]float64{rand.Float64() * xMax, rand.Float64() * yMax} // 随机生成点
// 			}

// 			np, I := nearestPoint(qRand) // 找到随机树中离 qRand 最近的节点
// 			newPoint := apfNewPoint(np, qRand) // 根据人工势场法生成新节点

// 			// 检查新节点与最近节点之间的路径是否与障碍物相交
// 			if noCollision(newPoint, np, obstacle11) && noCollision(newPoint, np, obstacle22) && noCollision(newPoint, np, obstacle33) {
// 				// 绘制路径
// 				line := plotter.XYs{{X: np[0], Y: np[1]}, {X: newPoint[0], Y: newPoint[1]}}
// 				pl, _ := plotter.NewLine(line)
// 				pl.Color = color.Black
// 				p.Add(pl)

// 				counter++
// 				pathV = append(pathV, [3]float64{newPoint[0], newPoint[1], float64(I)}) // 将新节点加入随机树
// 			}

// 			// 如果新节点接近目标点，则结束循环
// 			if math.Sqrt(math.Pow(newPoint[0]-goalPose[0], 2)+math.Pow(newPoint[1]-goalPose[1], 2)) <= bias {
// 				break
// 			}
// 		}

// 		// 绘制最终路径
// 		pose1 := pathV[len(pathV)-1]
// 		var path [][2]float64
// 		for pose1[2] > 0 {
// 			path = append([][2]float64{{pose1[0], pose1[1]}}, path...)
// 			pose1 = pathV[int(pose1[2])]
// 		}
// 		path = append([][2]float64{{pathV[0][0], pathV[0][1]}}, path...)

// 		// 绘制最终路径
// 		var finalPath plotter.XYs
// 		for _, point := range path {
// 			finalPath = append(finalPath, plotter.XY{X: point[0], Y: point[1]})
// 		}
// 		pl, _ := plotter.NewLine(finalPath)
// 		pl.Color = color.RGBA{255, 0, 0, 255} // 红色路径
// 		pl.LineStyle.Width = vg.Points(3)
// 		p.Add(pl)

// 		// 保存图像
// 		if err := p.Save(10*vg.Inch, 10*vg.Inch, fmt.Sprintf("rrt_path_%d.png", j)); err != nil {
// 			panic(err)
// 		}

// 		// 计算路径长度
// 		L := 0.0
// 		for i := 0; i < len(path)-1; i++ {
// 			L += math.Sqrt(math.Pow(path[i][0]-path[i+1][0], 2) + math.Pow(path[i][1]-path[i+1][1], 2))
// 		}

// 		// 记录实验结果
// 		Ll = append(Ll, L)
// 		Tt = append(Tt, time.Since(startTime).Seconds())
// 		Node = append(Node, len(path))
// 		All_Iterations = append(All_Iterations, numNodes)
// 	}

// 	// 输出实验结果
// 	fmt.Println("Average original path length:", mean(Ll))
// 	fmt.Println("Average experiment time:", mean(Tt))
// 	fmt.Println("Average number of experimental path nodes:", meanInt(Node))
// 	fmt.Println("Average number of experiment iterations:", meanInt(All_Iterations))
// }

// // 绘制障碍物
// func drawObstacle(p *plot.Plot, obstacle [4]float64, cl string) {
// 	pts := plotter.XYs{
// 		{X: obstacle[0], Y: obstacle[1]},
// 		{X: obstacle[0] + obstacle[2], Y: obstacle[1]},
// 		{X: obstacle[0] + obstacle[2], Y: obstacle[1] + obstacle[3]},
// 		{X: obstacle[0], Y: obstacle[1] + obstacle[3]},
// 		{X: obstacle[0], Y: obstacle[1]},
// 	}
// 	pl, _ := plotter.NewPolygon(pts)
// 	pl.Color = color.Black
// 	p.Add(pl)
// }

// // 计算平均值
// func mean(arr []float64) float64 {
// 	sum := 0.0
// 	for _, v := range arr {
// 		sum += v
// 	}
// 	return sum / float64(len(arr))
// }

// // 计算整数数组的平均值
// func meanInt(arr []int) float64 {
// 	sum := 0
// 	for _, v := range arr {
// 		sum += v
// 	}
// 	return float64(sum) / float64(len(arr))
// }

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
	Ll             []float64              // 存储每次运行的路径长度
	Tt             []float64              // 存储每次运行的时间
	Node           []int                  // 存储每次运行的路径节点数
	All_Iterations []int                  // 存储每次运行的迭代次数
	Escape_Count   []int                  // 存储每次运行成功逃脱局部极小值的次数
	goalPose       = [2]float64{999, 999} // 目标点坐标
	step           = 15.0                 // 步长减小以提高精度
	kp             = 1.0                  // 引力增益系数
	p0             = 50.0                // 斥力作用范围
	krep           = 0.5                  // 斥力增益系数
	bias           = 1.5 * step           // 允许的误差范围
	xMax           = 1000.0               // 地图的 X 轴最大值
	yMax           = 1000.0               // 地图的 Y 轴最大值
	obstacle1      = [4]float64{550, 150, 200, 200} // 障碍物 1 的坐标和大小 [x, y, width, height]
	obstacle2      = [4]float64{600, 550, 200, 200} // 障碍物 2 的坐标和大小
	obstacle3      = [4]float64{200, 200, 200, 300} // 障碍物 3 的坐标和大小
	obstacle11     [4]float64             // 障碍物 1 的扩展范围
	obstacle22     [4]float64             // 障碍物 2 的扩展范围
	obstacle33     [4]float64             // 障碍物 3 的扩展范围
	pathV          [][3]float64           // 存储随机树的节点 [x, y, parentIndex]
	startTime      time.Time              // 记录算法开始时间
)

func main() {
	rand.Seed(time.Now().UnixNano()) // 初始化随机数种子

	// 初始化障碍物的扩展范围
	or := 10.0
	obstacle11 = [4]float64{obstacle1[0] - or, obstacle1[1] - or, obstacle1[2] + 2*or, obstacle1[3] + 2*or}
	obstacle22 = [4]float64{obstacle2[0] - or, obstacle2[1] - or, obstacle2[2] + 2*or, obstacle2[3] + 2*or}
	obstacle33 = [4]float64{obstacle3[0] - or, obstacle3[1] - or, obstacle3[2] + 2*or, obstacle3[3] + 2*or}

	// 主循环，运行两次实验
	for j := 0; j < 2; j++ {
		// 创建绘图对象
		p := plot.New()
		p.Title.Text = "Improved RRT* Path Planning"
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

		startPose := [2]float64{0, 0} // 起点坐标
		pathV = [][3]float64{{startPose[0], startPose[1], -1}} // 初始化随机树，起点为根节点
		counter := 0
		numNodes := 5000 // 最大迭代次数
		pVal := 0.3      // 目标偏向概率
		escapeCount := 0 // 逃脱局部极小值的计数

		startTime = time.Now() // 记录开始时间

		// RRT 主循环
		for i := 0; i < numNodes; i++ {
			var qRand [2]float64
			if rand.Float64() <= pVal {
				qRand = goalPose // 以概率 pVal 选择目标点
			} else {
				qRand = [2]float64{rand.Float64() * xMax, rand.Float64() * yMax} // 随机生成点
			}

			np, I := nearestPoint(qRand) // 找到随机树中离 qRand 最近的节点
			newPoint := improvedAPFNewPoint(np, qRand) // 根据改进的人工势场法生成新节点

			// 检查新节点与最近节点之间的路径是否与障碍物相交
			if noCollision([2]float64{newPoint[0], newPoint[1]}, np, obstacle11) && noCollision([2]float64{newPoint[0], newPoint[1]}, np, obstacle22) && noCollision([2]float64{newPoint[0], newPoint[1]}, np, obstacle33) {
				// 绘制路径
				line := plotter.XYs{{X: np[0], Y: np[1]}, {X: newPoint[0], Y: newPoint[1]}}
				pl, _ := plotter.NewLine(line)
				pl.Color = color.Black
				p.Add(pl)

				counter++
				pathV = append(pathV, [3]float64{newPoint[0], newPoint[1], float64(I)}) // 将新节点加入随机树
			} else {
				// 如果发生碰撞，尝试使用逃逸力机制
				escapePoint := applyEscapeForce(np)
				if escapePoint != nil {
					escapeCount++
					// 绘制逃逸路径
					line := plotter.XYs{{X: np[0], Y: np[1]}, {X: escapePoint[0], Y: escapePoint[1]}}
					pl, _ := plotter.NewLine(line)
					pl.Color = color.RGBA{0, 0, 255, 255}
					p.Add(pl)
					pathV = append(pathV, [3]float64{escapePoint[0], escapePoint[1], float64(I)}) // 将逃逸点加入随机树
				}
			}

			// 如果新节点接近目标点，则结束循环
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
		pl.Color = color.RGBA{255, 0, 0, 255} // 红色路径
		pl.LineStyle.Width = vg.Points(3)
		p.Add(pl)

		// 保存图像
		if err := p.Save(10*vg.Inch, 10*vg.Inch, fmt.Sprintf("improved_rrt_path_%d.png", j)); err != nil {
			panic(err)
		}

		// 计算路径长度
		L := 0.0
		for i := 0; i < len(path)-1; i++ {
			L += math.Sqrt(math.Pow(path[i][0]-path[i+1][0], 2) + math.Pow(path[i][1]-path[i+1][1], 2))
		}

		// 记录实验结果
		Ll = append(Ll, L)
		Tt = append(Tt, time.Since(startTime).Seconds())
		Node = append(Node, len(path))
		All_Iterations = append(All_Iterations, numNodes)
		Escape_Count = append(Escape_Count, escapeCount)
	}

	// 输出实验结果
	fmt.Println("Average path length:", mean(Ll))
	fmt.Println("Average experiment time:", mean(Tt))
	fmt.Println("Average number of path nodes:", meanInt(Node))
	fmt.Println("Average number of iterations:", meanInt(All_Iterations))
	fmt.Println("Average escape count from local minima:", meanInt(Escape_Count))
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

// 计算平均值
func mean(arr []float64) float64 {
	sum := 0.0
	for _, v := range arr {
		sum += v
	}
	return sum / float64(len(arr))
}

// 计算整数数组的平均值
func meanInt(arr []int) float64 {
	sum := 0
	for _, v := range arr {
		sum += v
	}
	return float64(sum) / float64(len(arr))
}

// noCollision 检查两点之间的路径是否与障碍物相交
func noCollision(n2, n1 [2]float64, o [4]float64) bool {
	A := n1
	B := n2
	// 将障碍物的范围转换为四个角的坐标
	obs := [4]float64{o[0], o[1], o[0] + o[2], o[1] + o[3]}
	C1 := [2]float64{obs[0], obs[1]} // 左下角
	D1 := [2]float64{obs[0], obs[3]} // 左上角
	C2 := [2]float64{obs[0], obs[1]} // 左下角
	D2 := [2]float64{obs[2], obs[1]} // 右下角
	C3 := [2]float64{obs[2], obs[3]} // 右上角
	D3 := [2]float64{obs[2], obs[1]} // 右下角
	C4 := [2]float64{obs[2], obs[3]} // 右上角
	D4 := [2]float64{obs[0], obs[3]} // 左上角

	// 检查路径是否与障碍物的四条边相交
	ints1 := ccw(A, C1, D1) != ccw(B, C1, D1) && ccw(A, B, C1) != ccw(A, B, D1)
	ints2 := ccw(A, C2, D2) != ccw(B, C2, D2) && ccw(A, B, C2) != ccw(A, B, D2)
	ints3 := ccw(A, C3, D3) != ccw(B, C3, D3) && ccw(A, B, C3) != ccw(A, B, D3)
	ints4 := ccw(A, C4, D4) != ccw(B, C4, D4) && ccw(A, B, C4) != ccw(A, B, D4)

	// 如果路径与任何一条边相交，返回 false
	return !ints1 && !ints2 && !ints3 && !ints4
}

// ccw 判断三个点的方向（顺时针、逆时针或共线）
func ccw(A, B, C [2]float64) bool {
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
}

// nearestPoint 在随机树中找到离目标点最近的节点
func nearestPoint(randomPoint [2]float64) ([2]float64, int) {
	minDis := math.Inf(1) // 初始化最小距离为无穷大
	var theaDis [2]float64
	I := 0

	// 遍历随机树中的所有节点，找到距离目标点最近的节点
	for i := 0; i < len(pathV); i++ {
		dis := angelDist(randomPoint, [2]float64{pathV[i][0], pathV[i][1]})
		if dis < minDis {
			minDis = dis
			theaDis = [2]float64{pathV[i][0], pathV[i][1]}
			I = i
		}
	}

	return theaDis, I
}

// improvedAPFNewPoint 根据改进的人工势场法生成新的节点
func improvedAPFNewPoint(nearPoint, randomPoint [2]float64) []float64 {
	// 计算随机点与最近节点之间的距离
	len := math.Sqrt(math.Pow(randomPoint[0]-nearPoint[0], 2) + math.Pow(randomPoint[1]-nearPoint[1], 2))
	// 计算目标点与最近节点之间的距离
	len1 := math.Sqrt(math.Pow(goalPose[0]-nearPoint[0], 2) + math.Pow(goalPose[1]-nearPoint[1], 2))

	// 计算改进后的斥力
	F1 := improvedRepF(nearPoint, obstacle11)
	F2 := improvedRepF(nearPoint, obstacle22)
	F3 := improvedRepF(nearPoint, obstacle33)

	// 计算速度斥力势场
	vF1 := velocityRepF(nearPoint, obstacle11)
	vF2 := velocityRepF(nearPoint, obstacle22)
	vF3 := velocityRepF(nearPoint, obstacle33)

	// 合力为斥力与速度斥力之和
	Fx := F1[0] + F2[0] + F3[0] + vF1[0] + vF2[0] + vF3[0]
	Fy := F1[1] + F2[1] + F3[1] + vF1[1] + vF2[1] + vF3[1]

	// 如果合力为零，则合力方向为零
	if Fx == 0 && Fy == 0 {
		Fx = 0
		Fy = 0
	} else {
		// 归一化合力方向
		norm := math.Sqrt(Fx*Fx + Fy*Fy)
		Fx = Fx / norm
		Fy = Fy / norm
	}

	// 根据步长调整新节点的位置
	temStep := step
	if len < step {
		temStep = len
	}

	// 计算新节点的方向，结合随机点、目标点引力和斥力
	Ux := (randomPoint[0]-nearPoint[0])/len + kp*(goalPose[0]-nearPoint[0])/len1 + krep*Fx
	Uy := (randomPoint[1]-nearPoint[1])/len + kp*(goalPose[1]-nearPoint[1])/len1 + krep*Fy

	// 归一化方向向量
	normU := math.Sqrt(Ux*Ux + Uy*Uy)
	if normU == 0 {
		normU = 1
	}

	// 生成新节点
	newThea := []float64{
		nearPoint[0] + temStep*Ux/normU,
		nearPoint[1] + temStep*Uy/normU,
	}

	// 检查新节点是否在障碍物内部或过于接近障碍物
	if isInObstacle(newThea, obstacle11) || isInObstacle(newThea, obstacle22) || isInObstacle(newThea, obstacle33) {
		// 如果新节点在障碍物内部或过于接近障碍物，调整其位置
		newThea = adjustPointAwayFromObstacle(newThea)
	}

	return newThea
}

// improvedRepF 计算改进后的动态斥力势场
func improvedRepF(nearPoint [2]float64, obstacle [4]float64) [2]float64 {
	px := math.Sqrt(math.Pow(nearPoint[0]-(obstacle[0]+obstacle[2])/2, 2) + math.Pow(nearPoint[1]-(obstacle[1]+obstacle[3])/2, 2))
	if px > p0 {
		return [2]float64{0, 0}
	}

	// 动态调整斥力作用范围
	dynamicP0 := p0 * (1 - 0.5*math.Abs(math.Sin(time.Since(startTime).Seconds())))
	
	// 计算斥力分量
	repulsion := (1/px - 1/dynamicP0) / math.Pow(px, 2)
	
	// 计算方向向量
	dirX := (nearPoint[0] - (obstacle[0]+obstacle[2])/2) / px
	dirY := (nearPoint[1] - (obstacle[1]+obstacle[3])/2) / px

	return [2]float64{repulsion * dirX, repulsion * dirY}
}

// velocityRepF 计算速度斥力势场
func velocityRepF(nearPoint [2]float64, obstacle [4]float64) [2]float64 {
	// 假设障碍物的速度方向为随机（实际应用中应根据障碍物的实际速度计算）
	vx := rand.NormFloat64() * 10
	vy := rand.NormFloat64() * 10

	// 计算障碍物中心到节点的距离
	px := nearPoint[0] - (obstacle[0]+obstacle[2])/2
	py := nearPoint[1] - (obstacle[1]+obstacle[3])/2
	dist := math.Sqrt(px*px + py*py)

	// 如果距离大于作用范围，返回零向量
	if dist > p0 {
		return [2]float64{0, 0}
	}

	// 计算速度斥力分量
	kv := 0.5 // 速度斥力系数
	vRepulsion := kv * (vx*px + vy*py) / (dist * dist + 1e-6)

	// 计算方向向量
	dirX := px / dist
	dirY := py / dist

	return [2]float64{vRepulsion * dirX, vRepulsion * dirY}
}

// applyEscapeForce 应用逃逸力机制，尝试帮助机器人摆脱局部极小值
func applyEscapeForce(np [2]float64) []float64 {
	// 计算当前合力
	Fx, Fy := 0.0, 0.0
	F1 := improvedRepF(np, obstacle11)
	F2 := improvedRepF(np, obstacle22)
	F3 := improvedRepF(np, obstacle33)
	vF1 := velocityRepF(np, obstacle11)
	vF2 := velocityRepF(np, obstacle22)
	vF3 := velocityRepF(np, obstacle33)

	Fx = F1[0] + F2[0] + F3[0] + vF1[0] + vF2[0] + vF3[0]
	Fy = F1[1] + F2[1] + F3[1] + vF1[1] + vF2[1] + vF3[1]

	// 如果合力为零，无法应用逃逸力
	if Fx == 0 && Fy == 0 {
		return nil
	}

	// 计算逃逸力方向，垂直于合力方向
	escapeDirX := -Fy
	escapeDirY := Fx

	// 归一化逃逸力方向
	norm := math.Sqrt(escapeDirX*escapeDirX + escapeDirY*escapeDirY)
	if norm == 0 {
		return nil
	}
	escapeDirX /= norm
	escapeDirY /= norm

	// 应用逃逸力，生成逃逸点
	escapePoint := []float64{
		np[0] + step * escapeDirX,
		np[1] + step * escapeDirY,
	}

	// 检查逃逸点是否在障碍物内部或过于接近障碍物
	if isInObstacle(escapePoint, obstacle11) || isInObstacle(escapePoint, obstacle22) || isInObstacle(escapePoint, obstacle33) {
		// 如果逃逸点在障碍物内部或过于接近障碍物，调整其位置
		escapePoint = adjustPointAwayFromObstacle(escapePoint)
	}

	return escapePoint
}

// isInObstacle 检查点是否在障碍物内部或过于接近障碍物
func isInObstacle(point []float64, obstacle [4]float64) bool {
	// 检查点是否在障碍物内部
	if point[0] >= obstacle[0] && point[0] <= obstacle[0]+obstacle[2] &&
		point[1] >= obstacle[1] && point[1] <= obstacle[1]+obstacle[3] {
		return true
	}

	// 检查点是否过于接近障碍物
	distanceToObstacle := math.Sqrt(math.Pow(point[0]-(obstacle[0]+obstacle[2]/2), 2) + math.Pow(point[1]-(obstacle[1]+obstacle[3]/2), 2))
	if distanceToObstacle < 50.0 { // 设置安全距离
		return true
	}

	return false
}

// adjustPointAwayFromObstacle 调整点的位置，使其远离障碍物
func adjustPointAwayFromObstacle(point []float64) []float64 {
	// 计算点到各个障碍物的中心的距离
	dist1 := math.Sqrt(math.Pow(point[0]-(obstacle11[0]+obstacle11[2]/2), 2) + math.Pow(point[1]-(obstacle11[1]+obstacle11[3]/2), 2))
	dist2 := math.Sqrt(math.Pow(point[0]-(obstacle22[0]+obstacle22[2]/2), 2) + math.Pow(point[1]-(obstacle22[1]+obstacle22[3]/2), 2))
	dist3 := math.Sqrt(math.Pow(point[0]-(obstacle33[0]+obstacle33[2]/2), 2) + math.Pow(point[1]-(obstacle33[1]+obstacle33[3]/2), 2))

	// 找到最近的障碍物
	minDist := math.Min(math.Min(dist1, dist2), dist3)
	var closestObstacle [4]float64
	if minDist == dist1 {
		closestObstacle = obstacle11
	} else if minDist == dist2 {
		closestObstacle = obstacle22
	} else {
		closestObstacle = obstacle33
	}

	// 计算点与最近障碍物中心的向量
 vetorX := point[0] - (closestObstacle[0] + closestObstacle[2]/2)
 vetorY := point[1] - (closestObstacle[1] + closestObstacle[3]/2)

	// 归一化向量
	norm := math.Sqrt(vetorX*vetorX + vetorY*vetorY)
	if norm != 0 {
		vetorX /= norm
		vetorY /= norm
	}

	// 调整点的位置，使其远离障碍物
	adjustedPoint := []float64{
		point[0] + 5.0*vetorX, // 移动50个单位
		point[1] + 5.0*vetorY,
	}

	return adjustedPoint
}

func angelDist(pose1, pose2 [2]float64) float64 {
	return math.Sqrt(math.Pow(pose2[0]-pose1[0], 2) + math.Pow(pose2[1]-pose1[1], 2))
}