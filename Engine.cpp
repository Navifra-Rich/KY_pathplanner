#pragma once
#include "Engine.h"
#include <math.h>
long long dist(const Pos* p1, const Pos* p2) {
	return (long long)(p1->x - p2->x) * (p1->x - p2->x) + (long long)(p1->y - p2->y) * (p1->y - p2->y);
}
int ccw(const Pos* p1, const Pos* p2, const Pos* p3) {

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
int comparator(const Pos* left, const Pos* right, const Pos p) {
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
void QuickSort(vector<Pos>& a, int lo, int hi, Pos init) {
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
void Engine::setCarnum(int car_num) {
	this->car_num = car_num;
}
void Engine::setWaypointRand(int waypoint_num) {
	std::cout << "Way Points" << std::endl<<std::endl;
	//for (int i = 0; i < waypoint_num; i++) {
	//	Pos tmp;
	//	tmp.x = (double)(rand() % 100)+0.5;
	//	tmp.y = (double)(rand() % 100)+0.5;
	//	std::cout << "(" << tmp.x << "," << tmp.y << ")  ";
	//	this->wayPoints.push_back(tmp);
	//}

	// 오~각형
	this->wayPoints.push_back(Pos(2,0));
	this->wayPoints.push_back(Pos(1,2 ));
	this->wayPoints.push_back(Pos(4,0 ));
	this->wayPoints.push_back(Pos(5,2 ));
	this->wayPoints.push_back(Pos(3,4 ));


	// 사~각형
	//this->wayPoints.push_back(Pos(4, 0));
	//this->wayPoints.push_back(Pos(-4, 0));
	//this->wayPoints.push_back(Pos(7, 8));
	//this->wayPoints.push_back(Pos(-1, 8));

	// 정~사각형
	//this->wayPoints.push_back(Pos(0, 0));
	//this->wayPoints.push_back(Pos(8, 0));
	//this->wayPoints.push_back(Pos(0, 8));
	//this->wayPoints.push_back(Pos(8, 8));
	std::cout << std::endl;
}
vector<vector<Pos>> Engine::getPath2() {
	Pos center = this->getCenterPoint(this->wayPoints);

	std::cout << "Center = " << center.x << " " << center.y << std::endl;

	// ---------------------- Sort waypoint  ( Clock wise )
	std::cout << "------------ Start" << std::endl;
	for (vector<Pos>::iterator a = this->wayPoints.begin(); a != this->wayPoints.end(); a++) {
		std::cout << a->x << " " << a->y << std::endl;
	}
	std::cout << "------------ SOrt" << std::endl;
	QuickSort(this->wayPoints, 1, this->wayPoints.size() - 1, this->wayPoints[0]);
	for (vector<Pos>::iterator a = this->wayPoints.begin(); a != this->wayPoints.end(); a++) {
		std::cout << a->x << " " << a->y << std::endl;
	}

	double area = this->integral_sum(this->wayPoints);
	double area_per_unit = area / this->car_num;
	double area_th = 0.2;
	this->wayPoints.push_back(this->wayPoints[0]);

	vector<vector<Pos>> route;
	vector<Pos> mission_vec;
	mission_vec.push_back(center);
	mission_vec.push_back(this->wayPoints[0]);

	double cur_area = 0;		// 현재 넓이
	double cur_area_idx = 0;	// 현재 차량 번호
	double cur_vec_idx = 0;	// 현재 선분 번호
	std::cout << "PER AREA" << area_per_unit << std::endl;
	double tt = 0;
	for (vector<Pos>::iterator i = this->wayPoints.begin(); i < this->wayPoints.end()-1; i++) {
		Pos pivot = *i;
		Pos vec = *(i + 1) - *i;
		double dist = this->getDist(vec);

		// 10개로 선분 나눠서 넓이 계산
		for (int sub_vec_idx = 0; sub_vec_idx < 10; sub_vec_idx++) {
			Pos A, B;
			A = pivot + vec.mul(sub_vec_idx*0.1);
			B = pivot + vec.mul((sub_vec_idx+1)*0.1);
			double area = this->integral(center, A, B);
			tt += area;
			cur_area += area;
			//(area_per_unit * (1 - area_th) < cur_area) && (cur_area < area_per_unit* (1 + area_th))
			if ((area_per_unit < cur_area) &&
				(cur_area_idx + 1 != this->car_num)) {
				std::cout << "AREA CHANGFE !!" << std::endl;
				std::cout << B.x<<"  "<<B.y << std::endl;
				cur_area = 0;
				cur_area_idx++;
				mission_vec.push_back(B);
				route.push_back(mission_vec);
				for (vector<Pos> ::iterator j = mission_vec.begin(); j < mission_vec.end(); j++) {
					std::cout << "		Point " << j->x << " " << j->y << std::endl;
				}
				mission_vec.clear();
				vector<Pos>().swap(mission_vec);
				mission_vec.push_back(center);
				mission_vec.push_back(B);
			}
			//if (cur_area_idx + 1 == this->car_num) break;
		}
		//if (cur_area_idx + 1 == this->car_num) break;
		mission_vec.push_back(*(i + 1));
		cur_vec_idx++;
	}
	route.push_back(mission_vec);

	std::cout << "TOTOTOTAL  " << tt << std::endl;

	for (vector<vector<Pos>> ::iterator i = route.begin(); i < route.end(); i++) {
		std::cout << "ROUTE " << std::endl;
		for (vector<Pos> ::iterator j = i->begin(); j < i->end(); j++) {
			std::cout << "		Point " << j->x << " " << j->y << std::endl;
		}
	}
	return route;
	//std::cout << "AREA " << area << std::endl;
}
double Engine::getDist(Pos A) {
	return sqrt(A.x * A.x + A.y * A.y);
}
double Engine::inner(Pos A, Pos B) {
	return A.x * B.x + A.y * B.y;
}
double Engine::integral(Pos pivot, Pos a, Pos b){

	Pos A = a - pivot;
	Pos B = b - pivot;
	double distA = sqrt(A.x * A.x + A.y * A.y);
	double distB = sqrt(B.x * B.x + B.y * B.y);

	double inner = this->inner(A, B);
	double cos = inner / abs(distA * distB);
	double sin = sqrt(1 - cos * cos);

	//std::cout << "inner " << inner << std::endl;
	//std::cout << "COS " << cos << std::endl;
	//std::cout << "SIN " << sin << std::endl;
	double area = sin * distA * distB * 0.5;
	//std::cout << "DIST a " << distA << std::endl;
	//std::cout << "DIST b " << distB << std::endl;
	//std::cout << "AREA " << area << std::endl;

	return area;
}
double Engine::integral_sum(vector<Pos> poses) {
	Pos center = this->getCenterPoint(poses);
	poses.push_back(poses[0]);

	double area_total = 0;
	for (vector<Pos>::iterator i = poses.begin(); i < poses.end()-1; i++) {
		double area = this->integral(center, *i, *(i + 1));
		area_total += area;
	}
	std::cout << "AREA_TOTAL " << area_total<< std::endl;

	return area_total;
}
void Engine::getPath() {
	KMean_clustering km(this->car_num, this->wayPoints.size());
	this->wayPoints = km.clustering(wayPoints);
	std::cout << std::endl << " ---------------------------------------------------------- " << std::endl;
	std::cout << "      KMeans Result " << std::endl << std::endl;
	this->printWaypoints();

	//km.k = center of each waypoints group
	//ver  = center of each waypoints group (Pos type)
	Voronoi* vdg;
	for (int i = 0; i < km.K_COUNT; i++) {
		this->ver.push_back(new Pos(km.k[i].x, km.k[i].y));
		ver_val.push_back(Pos(km.k[i].x, km.k[i].y));
	}

	

	vdg = new Voronoi();
	double minY = 0;
	double maxY = 100;
	this->edges = vdg->ComputeVoronoiGraph(this->ver, minY, maxY);
	delete vdg;
	std::cout << std::endl << " --------------------------------------------------------- " << std::endl;
	std::cout << "      Voronoi Result " << std::endl << std::endl;
	for (vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
		std::cout << "(" << j->VertexA.x << ", " << j->VertexA.y << ")\t(" << j->VertexB.x << ", " << j->VertexB.y << ")\n";

	std::cout << " -------------------- " << std::endl;

	vector<vector<VEdge>> edges_vector;

	for (vector<Pos*>::iterator i = this->ver.begin(); i != this->ver.end(); i++)
	{
		vector<VEdge> edges;
		std::cout << "\t\tSite =  (" << (*i)->x << ", " << (*i)->y << ")\n";
		for (vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
		{
			if (((j->Left_Site.x == (*i)->x) && (j->Left_Site.y == (*i)->y)) || ((j->Right_Site.x == (*i)->x) && (j->Right_Site.y == (*i)->y)))
			{
				std::cout << "(" << j->VertexA.x << ", " << j->VertexA.y << ")\t(" << j->VertexB.x << ", " << j->VertexB.y << ")\n";
				edges.push_back(*j);
			}
		}
		edges_vector.push_back(edges);
		std::cout << "\n";
	}
	std::cout << std::endl << " --------------------------------------------------------- " << std::endl;
	std::cout << "      Convexhull  Result " << std::endl << std::endl;
	Pos center_pos;
	center_pos.x = 50;
	center_pos.y = 50;
	//getInnerPoints(ver_val, edges_vector, center_pos);
	//return 0;
}
vector<vector<VEdge>> Engine::waypoints2vector(vector<Pos> way) {
	vector<vector<VEdge>> edges_vector(this->car_num);
	vector<vector<Pos>> pos_vector(this->car_num);
	vector<Pos> area_center;	// Voronoi center

	//std::cout << "SIZE " << way.size() << std::endl;
	//std::cout << "SIZE " << this->car_num <<std::endl;
	
	for (vector<Pos>::iterator i = way.begin(); i != way.end(); i++) {
		//pos_vector[i->group].push_back(Pos(i->x, i->y));
		pos_vector[i->group].push_back(Pos(i->x, i->y));
	}
	for (int g = 0; g < this->car_num; g++) {
		int count = 0;
		area_center.push_back(Pos(0, 0));
		for (vector<Pos>::iterator i = this->wayPoints.begin(); i != this->wayPoints.end(); i++) {
			if (i->group == g) {
				count++;
				area_center[g].x += i->x;
				area_center[g].y += i->y;
			}
		}
		area_center[g].x /= count;
		area_center[g].y /= count;
	}
	//for (vector<Pos>::iterator k = area_center.begin(); k != area_center.end(); k++) {
	//	std::cout << "!! " << k->x << " " << k->y << std::endl;
	//}
	int group = 0;
	for (vector<Pos*>::iterator i = this->ver.begin(); i != this->ver.end(); i++)
	{
		for (vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
		{
			if (((j->Left_Site.x == (*i)->x) && (j->Left_Site.y == (*i)->y)) || ((j->Right_Site.x == (*i)->x) && (j->Right_Site.y == (*i)->y)))
			{
				float min_dist = 10000;
					Pos AB;
					Pos AC;
					Pos newPose;

					//for (vector<Pos>::iterator k = area_center.begin(); k != area_center.end(); k++) {
					Pos k = area_center[group];
					//std::cout << "k pose = " << k.x << "  " << k.y << std::endl;

					AB.x = j->VertexB.x - j->VertexA.x;
					AB.y = j->VertexB.y - j->VertexA.y;
					AC.x = j->VertexB.x - k.x;
					AC.y = j->VertexB.y - k.y;

					float inner = AB.x * AC.x + AB.y * AC.y;
					float inner_self = AB.x * AB.x + AB.y * AB.y;

					AB.x *= inner / inner_self;
					AB.y *= inner / inner_self;
					Pos tempPose;

					tempPose.x = j->VertexA.x + AB.x;
					tempPose.y = j->VertexA.y + AB.y;
					float dist = (tempPose.x - k.x) * (tempPose.x - k.x) + (tempPose.y - k.y) * (tempPose.y - k.y);
					if (dist < min_dist) {
						newPose = tempPose;
						min_dist = dist;
					}
				//std::cout << "Added pose = "<<newPose.x<<"  "<< newPose.y<< std::endl;
				pos_vector[group].push_back(newPose);
				//pos_vector[group].push_back(Pos((j->VertexA.x+ j->VertexB.x)/2, (j->VertexA.y+ j->VertexB.y)/2));
			}
		}
		//std::cout << "POSE VECTOR  "<<group << std::endl;
		for (vector<Pos>::iterator i = pos_vector[group].begin(); i != pos_vector[group].end(); i++) {
			//std::cout << "!! " << i->x << " " << i->y << std::endl;
		}
		group++;
	}
	group = 0;
	for (vector<vector<Pos>>::iterator i = pos_vector.begin(); i != pos_vector.end(); i++) {
		std::cout << "------------ Start" << std::endl;
		for (vector<Pos>::iterator a = i->begin(); a != i->end(); a++) {
			std::cout << a->x << " " << a->y << std::endl;
		}

		std::cout << "------------ SOrt" << std::endl;
		//QuickSort(*i, 1, i->size()-1);
		for (vector<Pos>::iterator a = i->begin(); a != i->end(); a++) {
			std::cout << a->x << " " << a->y << std::endl;
		}
		std::cout << "------------" << std::endl;
		for (vector<Pos>::iterator j = i->begin(); j != i->end(); j++) {
			VEdge  edges;
			edges.VertexA = Pos(j->x, j->y);

			if (j + 1 == i->end()) {
				edges.VertexB = Pos((i->begin())->x, (i->begin())->y);
			}
			else {
				edges.VertexB = Pos((j+1)->x, (j+1)->y);
			}
			edges_vector[group].push_back(edges);
		}
		group++;
	}
	for (vector<vector<VEdge>>::iterator i = edges_vector.begin(); i != edges_vector.end(); i++)
	{
		std::cout << "GROUP " << std::endl;

		for (vector<VEdge>::iterator j = i->begin(); j != i->end(); j++)
		{
			std::cout << "vector" << std::endl;
			std::cout << "		VECTOR " << j->VertexA.x << " " << j->VertexA.y << std::endl;
			std::cout << "		VECTOR " << j->VertexB.x << " " << j->VertexB.y << std::endl;
			//std::cout << "		VECTOR "<<j->VertexA << " " << " " << j->VertexB << std::endl;
		}
	}

	return edges_vector;
}
void Engine::printWaypoints() {
	for (int i = 0; i < wayPoints.size(); i++) {
		std::cout << "COORD = (" << wayPoints[i].x << " , " << wayPoints[i].y << ")  GROUP = " << wayPoints[i].group << std::endl;
	}
	/*for (int i = 0; i < K_COUNT; i++) {
		std::cout << "Group " << i << " = " << "(" << k[i].x << " , " << k[i].y << ")" << std::endl;
	}*/
}
Pos Engine::getCenterPoint(vector<Pos> poses) {
	Pos center;
	for (vector<Pos>::iterator i = poses.begin(); i < poses.end(); i++) {
		center.x += i->x;
		center.y += i->y;
	}
	center.x /= poses.size();
	center.y /= poses.size();
	return center;
}
Pos Engine::getMidPoint(VEdge edge) {
	Pos pos;
	pos.x = (edge.VertexA.x + edge.VertexB.x) / 2.0;
	pos.y = (edge.VertexA.y + edge.VertexB.y) / 2.0;
	return pos;
}
float innerProduct(Pos cluster_center, Pos edge_center, Pos center) {
	std::cout << "A " << cluster_center.x << "  " << cluster_center.y << std::endl;
	std::cout << "B " << edge_center.x << "  " << edge_center.y << std::endl;
	std::cout << "C " << center.x << "  " << center.y << std::endl;

	Pos vecA;
	Pos vecB;
	vecA.x = center.x - cluster_center.x;
	vecA.y = center.y - cluster_center.y;

	vecB.x = edge_center.x - cluster_center.x;
	vecB.y = edge_center.y - cluster_center.y;

	float th1;
	float th2;
	th1 = atan2(vecA.y, vecA.x) * 180 / 3.141592;
	th2 = atan2(vecB.y, vecB.x) * 180 / 3.141592;
	std::cout << "DEG = " << th1 << "  " << th2 << std::endl;
	float re_deg = abs(th1 - th2);
	while (re_deg >180)
		re_deg -= 180;
	std::cout << "REDEG = " << re_deg << std::endl;

	return re_deg;
}




// right가 left의 반시계 방향에 있으면 true이다.
// true if right is counterclockwise to left.
Pos p;


//int main(void) {
//
//	Pos ps[5];
//
//	ps[0].x = 2;    ps[0].y = 0;
//	ps[1].x = 1;    ps[1].y = 2;
//	ps[2].x = 4;    ps[2].y = 0;
//	ps[3].x = 5;    ps[3].y = 2;
//	ps[4].x = 3;    ps[4].y = 4;
//
//	p = ps[0];
//	QuickSort(ps, 1, 4);
//
//	for (int i = 0; i < 5; ++i) {
//		printf("(%d, %d)\n", ps[i].x, ps[i].y);
//	}
//
//}