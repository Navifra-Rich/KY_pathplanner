#include "PathPlanner.h"
#include "hgMath.h"
#include<iostream>

// vector<Pos> -> vector<vector<Pos> 분리 (그룹별로)
vector<vector<Pos>> PathPlanner::divide_waypoints(const vector<Pos> wayPoints) {
	int max_group = 0;
	vector<vector<Pos>> temp(100);
	for (vector<Pos>::const_iterator i = wayPoints.begin(); i < wayPoints.end(); i++) {
		temp[i->group].push_back(*i);
		if (i->group > max_group) max_group = i->group;
	}

	vector<vector<Pos>> devided_wayPoints(temp.begin(), temp.begin()+max_group+1);
	
	return devided_wayPoints;
	
}
// 폴리곤 내부에 유니폼 샘플링
vector<Pos> PathPlanner::getInnerPoint_polygon(vector<Pos> wayPoints, int width, int height) {
	vector<Pos> interPoints;
	double omi = 0.0001;
	wayPoints.push_back(wayPoints[0]);
	for (int i = -99; i < width; i+=1) {
		for (int j = -99; j < height; j+=1) {
			int intersect_count = 0;
			Pos here(i+omi, j);
			for (vector<Pos>::iterator it = wayPoints.begin(); it < wayPoints.end() - 1; it++) {
				Pos there(i + omi, j + 99999999);
				Pos intersect;

				bool isIntersected = hgMath::getIntersectPoint(*it, *(it + 1), here, there, &intersect);
				if (isIntersected) intersect_count++;
			}
			//if (intersect_count == 0) 

			if (intersect_count % 2 == 1) {
				//cout << "InterSected!! HERE!! " << here.x << " " << here.y << endl; 
				//<< "   INTER!! " << intersect.x << " " << intersect.y << endl;
				interPoints.push_back(here);
			}
			//else
				//cout << "NO!!!!!!!!!!!! HERE!! " << here.x << " " << here.y << endl;
		}
	}
	return interPoints;
}
// 적분 기반 웨이포인트 반환
vector<vector<Pos>> PathPlanner::divide_intergral_center(vector<Pos> wayPoints, double area_per_unit, int car_num) {
	vector<vector<Pos>> route;
	vector<Pos> mission_vec;
	Pos center = hgMath::getCenterPoint(wayPoints);
	wayPoints.push_back(wayPoints[0]);
	double cur_area = 0;		// 현재 넓이
	double cur_area_idx = 0;	// 현재 차량 번호
	double cur_vec_idx = 0;	// 현재 선분 번호
	mission_vec.push_back(center);
	mission_vec.push_back(wayPoints[0]);
	for (vector<Pos>::iterator i = wayPoints.begin(); i < wayPoints.end() - 1; i++) {
		Pos pivot = *i;
		Pos vec = *(i + 1) - *i;
		double dist = hgMath::getDist(vec);

		// 10개로 선분 나눠서 넓이 계산
		for (int sub_vec_idx = 0; sub_vec_idx < 100; sub_vec_idx++) {
			Pos A, B;
			A = pivot + vec.mul(sub_vec_idx * 0.01);
			B = pivot + vec.mul((sub_vec_idx + 1) * 0.01);
			double area = hgMath::integral(center, A, B);
			//tt += area;
			cur_area += area;
			//(area_per_unit * (1 - area_th) < cur_area) && (cur_area < area_per_unit* (1 + area_th))
			if ((area_per_unit < cur_area) &&
				(cur_area_idx + 1 != car_num)) {
				std::cout << "AREA CHANGFE !!" << std::endl;
				std::cout << B.x << "  " << B.y << std::endl;
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
	return route;
}

void PathPlanner::printWaypoints(vector<vector<Pos>> wayPoints) {
	for (vector<vector<Pos>> ::iterator i = wayPoints.begin(); i < wayPoints.end(); i++) {
		std::cout << "ROUTE " << std::endl;
		for (vector<Pos> ::iterator j = i->begin(); j < i->end(); j++) {
			std::cout << "		Point " << j->x << " " << j->y << std::endl;
		}
	}
}

