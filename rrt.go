package main

import (
	"math"
)

func noCollision(n2, n1 [2]float64, o [4]float64) bool {
	A := n1
	B := n2
	obs := [4]float64{o[0], o[1], o[0] + o[2], o[1] + o[3]}
	C1 := [2]float64{obs[0], obs[1]}
	D1 := [2]float64{obs[0], obs[3]}
	C2 := [2]float64{obs[0], obs[1]}
	D2 := [2]float64{obs[2], obs[1]}
	C3 := [2]float64{obs[2], obs[3]}
	D3 := [2]float64{obs[2], obs[1]}
	C4 := [2]float64{obs[2], obs[3]}
	D4 := [2]float64{obs[0], obs[3]}

	ints1 := ccw(A, C1, D1) != ccw(B, C1, D1) && ccw(A, B, C1) != ccw(A, B, D1)
	ints2 := ccw(A, C2, D2) != ccw(B, C2, D2) && ccw(A, B, C2) != ccw(A, B, D2)
	ints3 := ccw(A, C3, D3) != ccw(B, C3, D3) && ccw(A, B, C3) != ccw(A, B, D3)
	ints4 := ccw(A, C4, D4) != ccw(B, C4, D4) && ccw(A, B, C4) != ccw(A, B, D4)

	return !ints1 && !ints2 && !ints3 && !ints4
}

func ccw(A, B, C [2]float64) bool {
	return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])
}

func nearestPoint(randomPoint [2]float64) ([2]float64, int) {
	minDis := math.Inf(1)
	var theaDis [2]float64
	I := 0

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

func apfNewPoint(nearPoint, randomPoint [2]float64) [2]float64 {
	len := math.Sqrt(math.Pow(randomPoint[0]-nearPoint[0], 2) + math.Pow(randomPoint[1]-nearPoint[1], 2))
	len1 := math.Sqrt(math.Pow(goalPose[0]-nearPoint[0], 2) + math.Pow(goalPose[1]-nearPoint[1], 2))

	F1 := repF(nearPoint, randomPoint, obstacle11)
	F2 := repF(nearPoint, randomPoint, obstacle22)
	F3 := repF(nearPoint, randomPoint, obstacle33)

	var F [2]float64
	if F1[0]+F2[0]+F3[0] == 0 && F1[1]+F2[1]+F3[1] == 0 {
		F = [2]float64{0, 0}
	} else {
		F = [2]float64{krep * (F1[0] + F2[0] + F3[0]) / math.Sqrt(math.Pow(F1[0]+F2[0]+F3[0], 2)+math.Pow(F1[1]+F2[1]+F3[1], 2)), krep * (F1[1] + F2[1] + F3[1]) / math.Sqrt(math.Pow(F1[0]+F2[0]+F3[0], 2)+math.Pow(F1[1]+F2[1]+F3[1], 2))}
	}

	temStep := step
	if len < step {
		temStep = len
	}

	U := [2]float64{(randomPoint[0]-nearPoint[0])/len + kp*(goalPose[0]-nearPoint[0])/len1 + F[0], (randomPoint[1]-nearPoint[1])/len + kp*(goalPose[1]-nearPoint[1])/len1 + F[1]}
	newThea := [2]float64{nearPoint[0] + temStep*U[0]/math.Sqrt(math.Pow(U[0], 2)+math.Pow(U[1], 2)), nearPoint[1] +	temStep*U[1]/math.Sqrt(math.Pow(U[0], 2)+math.Pow(U[1], 2))}
	return newThea
}

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