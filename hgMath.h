#pragma once
#include<math.h>
#include"Pos.h"
#include<vector>
using namespace std;

class hgMath
{
private:
public:

	static double getDist(Pos A);
	static double inner(Pos A, Pos B);
	static double integral(Pos pivot, Pos a, Pos b);	// Integral triangle
	static double integral_sum(std::vector<Pos> poses);
	static Pos getCenterPoint(std::vector<Pos> poses);
	static bool getIntersectPoint(const Pos& AP1, const Pos& AP2, const Pos& BP1, const Pos& BP2, Pos* IP);// Whether 2 line intersect
	static std::vector<Pos> PCA(std::vector<Pos>);

};
