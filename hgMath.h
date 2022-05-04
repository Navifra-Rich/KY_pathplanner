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
	static void QuickSort(std::vector<Pos>& a, int lo, int hi, Pos init);
	static std::vector<Pos> PCA(std::vector<Pos>);
	static std::vector<Pos> makeConvex(const std::vector<Pos>);


	// In Quick sort
	static long long dist(const Pos* p1, const Pos* p2);
	static int comparator(const Pos* left, const Pos* right, const Pos p);
	static int ccw(const Pos* p1, const Pos* p2, const Pos* p3);

};
