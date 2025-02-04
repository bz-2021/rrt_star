package main

import (
	"math"
)

// noCollision 检查两点之间的路径是否与障碍物相交
// 参数：
//   - n1, n2: 两个点的坐标 [x, y]
//   - o: 障碍物的范围 [x, y, width, height]
// 返回值：
//   - bool: 如果路径不与障碍物相交，返回 true；否则返回 false
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
// 参数：
//   - A, B, C: 三个点的坐标 [x, y]
// 返回值：
//   - bool: 如果方向为逆时针，返回 true；否则返回 false
func ccw(A, B, C [2]float64) bool {
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
}

// nearestPoint 在随机树中找到离目标点最近的节点
// 参数：
//   - randomPoint: 目标点的坐标 [x, y]
// 返回值：
//   - [2]float64: 最近节点的坐标 [x, y]
//   - int: 最近节点在 pathV 中的索引
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

// apfNewPoint 根据人工势场法生成新的节点
// 参数：
//   - nearPoint: 最近节点的坐标 [x, y]
//   - randomPoint: 随机点的坐标 [x, y]
// 返回值：
//   - [2]float64: 新生成的节点坐标 [x, y]
func apfNewPoint(nearPoint, randomPoint [2]float64) [2]float64 {
	// 计算随机点与最近节点之间的距离
	len := math.Sqrt(math.Pow(randomPoint[0]-nearPoint[0], 2) + math.Pow(randomPoint[1]-nearPoint[1], 2))
	// 计算目标点与最近节点之间的距离
	len1 := math.Sqrt(math.Pow(goalPose[0]-nearPoint[0], 2) + math.Pow(goalPose[1]-nearPoint[1], 2))

	// 计算障碍物的斥力
	F1 := repF(nearPoint, randomPoint, obstacle11)
	F2 := repF(nearPoint, randomPoint, obstacle22)
	F3 := repF(nearPoint, randomPoint, obstacle33)

	var F [2]float64
	// 如果斥力为零，则合力为零
	if F1[0]+F2[0]+F3[0] == 0 && F1[1]+F2[1]+F3[1] == 0 {
		F = [2]float64{0, 0}
	} else {
		// 计算合力的方向
		F = [2]float64{
			krep * (F1[0] + F2[0] + F3[0]) / math.Sqrt(math.Pow(F1[0]+F2[0]+F3[0], 2)+math.Pow(F1[1]+F2[1]+F3[1], 2)),
			krep * (F1[1] + F2[1] + F3[1]) / math.Sqrt(math.Pow(F1[0]+F2[0]+F3[0], 2)+math.Pow(F1[1]+F2[1]+F3[1], 2)),
		}
	}

	// 根据步长调整新节点的位置
	temStep := step
	if len < step {
		temStep = len
	}

	// 计算新节点的方向
	U := [2]float64{
		(randomPoint[0]-nearPoint[0])/len + kp*(goalPose[0]-nearPoint[0])/len1 + F[0],
		(randomPoint[1]-nearPoint[1])/len + kp*(goalPose[1]-nearPoint[1])/len1 + F[1],
	}
	// 生成新节点
	newThea := [2]float64{
		nearPoint[0] + temStep*U[0]/math.Sqrt(math.Pow(U[0], 2)+math.Pow(U[1], 2)),
		nearPoint[1] + temStep*U[1]/math.Sqrt(math.Pow(U[0], 2)+math.Pow(U[1], 2)),
	}
	return newThea
}

// repF 计算障碍物的斥力
// 参数：
//   - nearPoint: 最近节点的坐标 [x, y]
//   - randomPoint: 随机点的坐标 [x, y]
//   - obstacle: 障碍物的范围 [x, y, width, height]
// 返回值：
//   - [2]float64: 斥力的向量 [Fx, Fy]
func repF(nearPoint, randomPoint [2]float64, obstacle [4]float64) [2]float64 {
	px := math.Sqrt(math.Pow(nearPoint[0]-(obstacle[0]+obstacle[2])/2, 2) + math.Pow(nearPoint[1]-(obstacle[1]+obstacle[3])/2, 2))
	if px > p0 {
		return [2]float64{0, 0}
	}
	return [2]float64{(1/px - 1/p0) / math.Pow(px, 2) * (nearPoint[0] - (obstacle[0]+obstacle[2])/2) / px, (1/px - 1/p0) / math.Pow(px, 2) * (nearPoint[1] - (obstacle[1]+obstacle[3])/2) / px}
}

func angelDist(pose1, pose2 [2]float64) float64 {
	return math.Sqrt(math.Pow(pose2[0]-pose1[0], 2) + math.Pow(pose2[1]-pose1[1], 2))
}