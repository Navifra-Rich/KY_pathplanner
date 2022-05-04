#include "hgMath.h"
#include<iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
long long hgMath::dist(const Pos* p1, const Pos* p2) {
	return (long long)(p1->x - p2->x) * (p1->x - p2->x) + (long long)(p1->y - p2->y) * (p1->y - p2->y);
}
int hgMath::ccw(const Pos* p1, const Pos* p2, const Pos* p3) {

	int cross_product = (p2->x - p1->x) * (p3->y - p1->y) - (p3->x - p1->x) * (p2->y - p1->y);
	if (cross_product > 0) {
		return 1;
	}
	else if (cross_product < 0) {
		return -1;
	}
	else {
		return 0;
	}
}
int hgMath::comparator(const Pos* left, const Pos* right, const Pos p) {
	int ret;
	int direction = ccw(&p, left, right);
	if (direction == 0) {
		ret = (dist(&p, left) < dist(&p, right));
	}
	else if (direction == 1) {
		ret = 1;
	}
	else {
		ret = 0;
	}
	return ret;
}
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
void quickSortByAngle(int first, int last, Pos* p)
{
	if (first >= last) return;

	int pivot = first;
	int i = first + 1;
	int j = last;

	while (i <= j)
	{
		while (comp(p[i], p[pivot], p) && i <= last) i++;
		while (!comp(p[j], p[pivot], p) && j > first) j--;

		if (i >= j) break;

		Pos tmp = p[i];
		p[i] = p[j];
		p[j] = tmp;
	}

	Pos tmp = p[j];
	p[j] = p[pivot];
	p[pivot] = tmp;

	quickSortByAngle(first, j - 1, p);
	quickSortByAngle(j + 1, last, p);
}

void hgMath::QuickSort(std::vector<Pos>& a, int lo, int hi, Pos init) {
	if (hi - lo <= 0) {
		return;
	}

	// 현재 배열 범위의 중앙값을 피벗으로 선택한다.
	// Select the median as pivot in the current array range.
	Pos pivot = a[lo + (hi - lo + 1) / 2];
	int i = lo, j = hi;

	// 정복 과정
	// Conquer process
	while (i <= j) {
		// 피벗의 왼쪽에는 comparator(타겟, "피벗")을 만족하지 않는 인덱스를 선택 (i)
		// On the left side of the pivot, select an index that doesn't satisfy the comparator(target, "pivot"). (i)
		while (comparator(&a[i], &pivot, init)) i++;

		// 피벗의 오른쪽에는 comparator("피벗", 타겟)을 만족하지 않는 인덱스를 선택 (j)
		// On the right side of the pivot, select an index that doesn't satisfy the comparator("pivot", target). (j)
		while (comparator(&pivot, &a[j], init)) j--;
		// (i > j) 피벗의 왼쪽에는 모든 값이 피벗보다 작고 피벗의 오른쪽에는 모든 값이 피벗보다 큰 상태가 되었음.
		// (i > j) On the left side of the pivot, all values are smaller than the pivot, and on the right side of the pivot, all values are larger than the pivot.
		if (i > j) {
			break;
		}

		// i번째 값은 피벗 보다 크고 j번째 값은 피벗보다 작으므로 두 값을 스왑한다.
		// The i-th value is larger than the pivot and the j-th value is smaller than the pivot, so swap the two values.
		Pos temp = a[i];
		a[i] = a[j];
		a[j] = temp;

		// 인덱스 i를 1증가 시키고 인덱스 j를 1 감소 시켜서 탐색 범위를 안쪽으로 좁힌다.
		// Narrow the search inward by increasing index i by one and decreasing index j by one.
		i++;
		j--;
	}

	// 분할 과정
	// Divide process
	QuickSort(a, lo, j, init);
	QuickSort(a, i, hi, init);
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
	//points.clear();
	//points.push_back(Pos(170, 70));
	//points.push_back(Pos(150, 45));
	//points.push_back(Pos(160, 55));
	//points.push_back(Pos(180, 60));
	//points.push_back(Pos(172, 80));

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


std::vector<Pos> hgMath::makeConvex(std::vector<Pos> p) {
	int size = p.size();
	std::cout << "INXONVEX" << std::endl;
	int stack[99999];
	double minX = 1000000000, minY = 1000000000;
	int minIdx = 0;
	for (int i = 0; i < size; i++) {

		if (minY > p[i].y || (minY == p[i].y && minX > p[i].x))
		{
			minX = p[i].x;
			minY = p[i].y;
			minIdx = i;
		}
	}
	p[minIdx].x = p[0].x;
	p[minIdx].y = p[0].y;
	p[0].x = minX;
	p[0].y = minY;

	quickSortByAngle(1, size - 1, &p[0]);
	int idx = -1;
	stack[++idx] = 0;
	stack[++idx] = 1;

	int next = 2;
	while (next < size)
	{

		while ((idx + 1) >= 2)
		{
			int second = stack[idx--];
			int first = stack[idx];

			if (leftTurn(p[first], p[second], p[next]))
			{
				stack[++idx] = second;
				break;
			}
		}
		stack[++idx] = next++;
		// std::cout<<"idx = "<<idx<<" next "<<next-1<<std::endl;

	}
	std::vector<Pos> stack2;
	for (int i = 0; i < idx; i++) {
		//std::cout << "IDX " << stack[i] << std::endl;
		//std::cout << "		 " << poses[stack[i]].x<<" "<< poses[stack[i]].y << std::endl;
		stack2.push_back(p[stack[i]]);
	}
	return stack2;

	//convec_publish(p, stack, idx);


	//int stack[1000];
	//std::vector<Pos> stack2;
	//Pos minPose(99999,99999);
	//for (std::vector<Pos>::iterator it = poses.begin(); it < poses.end(); it++) {
	//	if (it->y < minPose.y or (it->y == minPose.y and it->x < minPose.x)) {
	//		minPose.y = it->y;
	//		minPose.x = it->x;
	//	}
	//}
	//std::cout << "POses " << poses[0].x << " " << poses[0].y << std::endl;
	//poses[0] = minPose;
	//std::cout << "POses " << poses[0].x << " " << poses[0].y << std::endl;
	//hgMath::QuickSort(poses, 1, poses.size()-1, poses[0]);
	//int idx = -1;
	//stack[++idx] = 0;
	//stack[++idx] = 1;
	//
	//int next = 2;
	//while (next < poses.size())
	//{
	//	while ((idx + 1) >= 2)
	//	{
	//		int second = stack[idx--];
	//		int first = stack[idx];
	//		//std::cout << "FIRST " << poses[first].x << " " << poses[first].y << std::endl;
	//		//std::cout << "Secon " << poses[second].x << " " << poses[second].y << std::endl;
	//		//std::cout << "Next  " << poses[next].x << " " << poses[next].y << std::endl;
	//		if (hgMath::ccw(&poses[first], &poses[second], &poses[next])>0)
	//		{
	//			stack[++idx] = second;
	//			break;
	//		}
	//	}
	//	stack[++idx] = next++;
	//}
	//for (int i = 0; i < idx; i++) {
	//	//std::cout << "IDX " << stack[i] << std::endl;
	//	//std::cout << "		 " << poses[stack[i]].x<<" "<< poses[stack[i]].y << std::endl;
	//	stack2.push_back(poses[stack[i]]);
	//}
	//return stack2;
	////return idx + 1;
}

//vector<int> hgMath::makeConvex(const vector<Pos> poses){
//
//	int stack[100];
//	int idx = -1;
//	stack[++idx] = 0;
//	stack[++idx] = 1;
//
//	int next = 2;
//	while (next < poses.size())
//	{
//
//		while ((idx + 1) >= 2)
//		{
//			int second = stack[idx--];
//			int first = stack[idx];
//
//			if (hgMath::ccw(&poses[first], &poses[second], &poses[next]))
//			{
//				stack[++idx] = second;
//				break;
//			}
//		}
//		stack[++idx] = next++;
//		// std::cout<<"idx = "<<idx<<" next "<<next-1<<std::endl;
//
//	}
//	for (int i = 0; i < idx; i++) {
//		std::cout << "IDX " << stack[i] << std::endl;
//	}
//	vector<int> stack2;
//	return stack2;
//
//
//
//	//int idx = 1;
//	//vector<int> stack;
//	//stack.push_back(0);
//	//stack.push_back(1);
//	////stack[++idx] = 1;
//	//
//	//int next = 2;
//	//while (next < poses.size())
//	//{
//	//
//	//	while ((idx + 1) >= 2)
//	//	{
//	//		int second = stack[idx--];
//	//		int first = stack[idx];
//	//
//	//		if (hgMath::ccw(&poses[first], &poses[second], &poses[next]) > 0)
//	//		//if (leftTurn(p[first], p[second], p[next]))
//	//		{
//	//			stack.push_back(second);
//	//			idx++;
//	//			break;
//	//		}
//	//	}
//	//	idx++;
//	//	stack.push_back(next);
//	//	next++;
//	//	// std::cout<<"idx = "<<idx<<" next "<<next-1<<std::endl;
//	//}
//	//return stack;
//}