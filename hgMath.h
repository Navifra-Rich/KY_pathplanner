#pragma once
#include<math.h>
#include"Pos.h"
#include<vector>
using namespace std;

class hgMath
{
private:
public:

	static double getDist(Pos A);						// Get Euclidian distance
	static double inner(Pos A, Pos B);					// Inner product
	static double integral_sum(std::vector<Pos> poses);	// Integral polygon
	static double integral(Pos pivot, Pos a, Pos b);	// Integral by dividing each point into triangles
	static Pos getCenterPoint(std::vector<Pos> poses);	// Get center of vectors
	static bool getIntersectPoint(const Pos& AP1, const Pos& AP2, const Pos& BP1, const Pos& BP2, Pos* IP);// Whether 2 line intersect
	static std::vector<Pos> PCA(std::vector<Pos>);

};
