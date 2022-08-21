#include "hgMath.h"
#include<iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"

double ccw(Pos& A, Pos& B, Pos& C) {
	Pos BO = B - A;
	Pos CO = C - A;
	return ((double)BO.x * (double)CO.y - (double)BO.y * (double)CO.x);
}
bool leftTurn(Pos& A, Pos& B, Pos& C) {

	return ccw(A, B, C) > 0;
}
bool comp(Pos& A, Pos& B, Pos* p)
{
	Pos AO = A - p[0];
	Pos BO = B - p[0];

	double outerProduct = ((double)AO.x * (double)BO.y - (double)AO.y * (double)BO.x);
	if (outerProduct != 0)
		return (outerProduct > 0);
	if (A.y != B.y)
		return A.y < B.y;
	return A.x < B.x;
}
double hgMath::getDist(Pos A) {
	return sqrt(A.x * A.x + A.y * A.y);
}
double hgMath::inner(Pos A, Pos B) {
	return A.x * B.x + A.y * B.y;
}
double hgMath::integral(Pos pivot, Pos a, Pos b) {

	Pos A = a - pivot;
	Pos B = b - pivot;
	double distA = sqrt(A.x * A.x + A.y * A.y);
	double distB = sqrt(B.x * B.x + B.y * B.y);

	double inner = hgMath::inner(A, B);
	double cos = inner / abs(distA * distB);
	double sin = sqrt(1 - cos * cos);
	double area = sin * distA * distB * 0.5;

	return area;
}
double hgMath::integral_sum(vector<Pos> poses) {
	
	Pos center = hgMath::getCenterPoint(poses);
	poses.push_back(poses[0]);

	double area_total = 0;
	for (std::vector<Pos>::iterator i = poses.begin(); i < poses.end() - 1; i++) {
		double area = hgMath::integral(center, *i, *(i + 1));
		area_total += area;
	}
	//std::cout << "AREA_TOTAL " << area_total << std::endl;

	return area_total;
}
Pos hgMath::getCenterPoint(vector<Pos> poses) {
	Pos center;
	for (std::vector<Pos>::iterator i = poses.begin(); i < poses.end(); i++) {
		center.x += i->x;
		center.y += i->y;
	}
	center.x /= poses.size();
	center.y /= poses.size();
	return center;
}
bool hgMath::getIntersectPoint(const Pos& AP1, const Pos& AP2,
	const Pos& BP1, const Pos& BP2, Pos* IP)
{
	double t;
	double s;
	double under = (BP2.y - BP1.y) * (AP2.x - AP1.x) - (BP2.x - BP1.x) * (AP2.y - AP1.y);
	if (under == 0) return false;

	double _t = (BP2.x - BP1.x) * (AP1.y - BP1.y) - (BP2.y - BP1.y) * (AP1.x - BP1.x);
	double _s = (AP2.x - AP1.x) * (AP1.y - BP1.y) - (AP2.y - AP1.y) * (AP1.x - BP1.x);

	t = _t / under;
	s = _s / under;

	if (t <= 0.0 || t>=1.0 || s <= 0.0 || s>=1.0) return false;
	if (_t == 0 && _s == 0) return false;

	IP->x = AP1.x + t * (double)(AP2.x - AP1.x);
	IP->y = AP1.y + t * (double)(AP2.y - AP1.y);

	return true;
}
std::vector<Pos> hgMath::PCA(std::vector<Pos> points) {

	int n = points.size();
	std::vector<Pos> pca;
	double a, b, c, d, x_sum, y_sum;
	a = b = c = d = x_sum = y_sum = 0;
	for (int i = 0; i < n; i++) {
		x_sum += points[i].x;
		y_sum += points[i].y;
	}
	double x_mean = x_sum / n;
	double y_mean = y_sum / n;
	for (int i = 0; i < n; i++){
		a += (points[i].x-x_mean) * (points[i].x-x_mean);
		b += (points[i].x-x_mean) * (points[i].y-y_mean);
		c += (points[i].y-y_mean) * (points[i].x-x_mean);
		d += (points[i].y-y_mean) * (points[i].y-y_mean);
	}
	std::cout << x_mean << " " << y_mean << std::endl;
	std::cout << a << " " << b << " " << c << " " << d << std::endl;
	Pos pc1(a / n, b / n);
	Pos pc2(c / n, d / n);
	pca.push_back(pc1);
	pca.push_back(pc2);
	std::cout << " PCA " << pca[0].x << "  " << pca[0].y << "  " << pca[1].x << "  " << pca[1].y << std::endl;
	return pca;
}

