#pragma once
#include "Engine.h"
#include <math.h>
#include "hgMath.h"
#include "PathPlanner.h"
#include <fstream>
#include <time.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
void Engine::setCarnum(int car_num) {
	this->car_num = car_num;
}
void Engine::setWaypointRand(int waypoint_num) {
	std::cout << "Way Points" << std::endl<<std::endl;
	//3개: (280, 475)  (150.096, 250)  (409.904, 250)
	//	4개 : (280, 475)  (130, 325)  (280, 175)  (430, 325)
	//	5개 : (280, 475)  (137.342, 371.353)  (191.832, 203.647)  (368.168, 203.647)  (422.658, 371.353)
	//	6개 : (280, 475)  (150.096, 400)  (150.096, 250)  (280, 175)  (409.904, 250)  (409.904, 400)
	// 오~각형
	//this->wayPoints.push_back(Pos(200,0));
	//this->wayPoints.push_back(Pos(400,0));
	//this->wayPoints.push_back(Pos(500,200 ));
	//this->wayPoints.push_back(Pos(300,400 ));
	//this->wayPoints.push_back(Pos(100,200));

	// 사~각형
	//this->wayPoints.push_back(Pos(120, 50));
	//this->wayPoints.push_back(Pos(280, 50));
	//this->wayPoints.push_back(Pos(340, 210));
	//this->wayPoints.push_back(Pos(280, 190));
	//this->wayPoints.push_back(Pos(180, 210));
	// 
	// 마룸몽
	//this->wayPoints.push_back(Pos(280, 503));
	//this->wayPoints.push_back(Pos(130, 353));
	//this->wayPoints.push_back(Pos(280, 203));
	//this->wayPoints.push_back(Pos(430, 353));
	// 
	// 정~사각형
	//this->wayPoints.push_back(Pos(0, 0));
	//this->wayPoints.push_back(Pos(20, 0));
	//this->wayPoints.push_back(Pos(20, 20));
	//this->wayPoints.push_back(Pos(0, 20));

	// 길쭊사각형
	//this->wayPoints.push_back(Pos(10, 10));
	//this->wayPoints.push_back(Pos(110, 10));
	//this->wayPoints.push_back(Pos(110, 600));
	//this->wayPoints.push_back(Pos(10, 600));

	// 별별
	this->wayPoints.push_back(Pos(100, 0));
	this->wayPoints.push_back(Pos(200, 200));
	this->wayPoints.push_back(Pos(300, 0));
	this->wayPoints.push_back(Pos(270, 200));
	this->wayPoints.push_back(Pos(400, 400));
	this->wayPoints.push_back(Pos(250, 400));
	this->wayPoints.push_back(Pos(200, 600));
	this->wayPoints.push_back(Pos(150, 400));
	this->wayPoints.push_back(Pos(0, 400));
	this->wayPoints.push_back(Pos(130, 200));



	//NOP 
	//this->wayPoints.push_back(Pos(280, 475));
	//this->wayPoints.push_back(Pos(130, 325));
	//this->wayPoints.push_back(Pos(280, 175));
	//this->wayPoints.push_back(Pos(430, 325));

	//this->wayPoints.push_back(Pos(280, 475));
	//this->wayPoints.push_back(Pos(150, 400));
	//this->wayPoints.push_back(Pos(150, 250));
	//this->wayPoints.push_back(Pos(280, 175));
	//this->wayPoints.push_back(Pos(410, 250));
	//this->wayPoints.push_back(Pos(700, 400));
}
std::vector<std::vector<Pos>> Engine::getPath2() {

	Pos center = hgMath::getCenterPoint(this->wayPoints);

	std::cout << "Center = " << center.x << " " << center.y << std::endl;

	//PathPlanner::printPathpoints(this->wayPoints);
	//PathPlanner::printPathpoints(this->wayPoints);

	std::vector<Pos> inter_points = PathPlanner::getInnerPoint_polygon(wayPoints);
	std::cout << inter_points[0].x << " " << inter_points[0].y << std::endl;
	std::cout << inter_points[0].x << " " << inter_points[0].y << std::endl;
	std::cout << "N = " << inter_points.size() << std::endl;

	std::vector<Pos> pca = hgMath::PCA(inter_points);
	if (pca[0].x == 0) pca[0].x += 0.000001;
	if (pca[1].y == 0) pca[1].y += 0.000001;
	double ratio = pca[0].x / (pca[1].y);
	if (ratio < 1) ratio = 1 / (ratio);

	std::cout << "RATIO " << ratio << std::endl;

	std::vector<std::vector<Pos>> route;
	std::vector<std::vector<Pos>> route_integral;
	//return route;
	// 공분산 큰 경우
	if (true) {
		std::cout << "Kmeans mode" << std::endl;
	// if (ratio > 0) {
		//KMean_clustering km(this->car_num, wayPoints.size());
		//this->wayPoints = km.clustering(wayPoints);
		clock_t start, end;
		start = clock(); 
		KMean_clustering km(this->car_num, inter_points.size());
		//this->wayPoints = km.clustering(inter_points);
		end = clock();
		cout << "TIME!!!!!!!!! = " << (double)(end - start) << endl;
		//return route;
		//std::cout << "      KMeans Result " << std::endl << std::endl;
		std::vector<std::vector<Pos>> bf_convex_route;
		bf_convex_route = PathPlanner::divide_waypoints(km.clustering(inter_points));
		
		//PathPlanner::printWaypoints(bf_convex_route);
		//cv::Mat My_Mat1(500, 500, CV_8SC1, cv::Scalar::all(0));
		//PathPlanner::draw_Contours();
		int temp[500][500];
		int temp_idx = 0;
		for (std::vector<std::vector<Pos>>::iterator it = bf_convex_route.begin(); it < bf_convex_route.end(); it++) {
			//hgMath::QuickSort(*it, 1, it->size()-1, (*it)[0]);
			cout << "SIZE ======= "<<(*it).size() << endl;
			//std::vector<Pos> convex_route = hgMath::makeConvex(*it);
			std::vector<Pos> convex_route = PathPlanner::save_clustering_img(*it);
			route.push_back(convex_route);

			//for (std::vector<Pos>::iterator itt = it->begin(); itt < it->end(); itt++) {
			//	temp[int(itt->y)][int(itt->x)] = 0;
			//}
			//temp_idx++;
		}

		std::cout << "CARACACAR   " << this->car_num << std::endl;
		std::cout << "SIXZER   " << route.size() << std::endl;
		std::cout << "SIXZER   " << route.size() << std::endl;
		std::cout << "SIXZER   " << bf_convex_route.size() << std::endl;
		std::cout << "SIXZER   " << bf_convex_route.size() << std::endl;

	}
	//return route;
	// 일반적인 경우
	// else {
	if(true){
		std::cout << "Integral" << std::endl;
		clock_t start, end;
		start = clock();
		double area = hgMath::integral_sum(this->wayPoints);
		std::cout << "Area " << area << "Car num"<<this->car_num<<std::endl;
		double area_per_unit = area / this->car_num;
		double area_th = 0.2;
		route_integral = PathPlanner::divide_intergral_center(this->wayPoints, area_per_unit, this->car_num);
		end = clock();
		cout << "TIME!!!!!!!!! = " << (double)(end - start) << endl;

	}
	//for (std::vector<std::vector<Pos>>::iterator it = route_integral.begin(); it < route.end(); it++) {
	//	it->push_back(it->at(0));
	//}
	cout << "Len = " << route.size()<< endl;
	cout << "Len = " << route_integral.size()<< endl;

	 //PathPlanner::printWaypoints(route_integral);
	route = PathPlanner::samplingAllRoute(route);
	route_integral = PathPlanner::samplingAllRoute(route_integral);

	std::vector<std::vector<float>> cur1 = PathPlanner::getCurvatureFromRoute(route);
	cout << "-----------------" << endl;
	std::vector<std::vector<float>> cur2 = PathPlanner::getCurvatureFromRoute(route_integral);
	PathPlanner::printWaypoints(route, cur1);

	float max_cur1, max_cur2;
	max_cur1 = max_cur2 = 0;
	for (int i = 0; i < cur1.size(); i++) {
		for (int j = 0; j < cur1[i].size(); j++) if (cur1[i][j] > max_cur1) max_cur1 = cur1[i][j];
		for (int j = 0; j < cur2[i].size(); j++) if (cur2[i][j] > max_cur2) max_cur2 = cur2[i][j];
	}
	std::cout << "CUR1 " << max_cur1 << " CUR2  " << max_cur2 << std::endl;
	for (int i = 0; i < route.size(); i++) {
	std::cout << "CS " << cur1[i].size() << " RS  " << route[i].size() << std::endl;
	}
	return route_integral;
}

// Unused
std::vector<std::vector<VEdge>> Engine::waypoints2vector(std::vector<Pos> way) {
	std::vector<std::vector<VEdge>> edges_vector(this->car_num);
	std::vector<std::vector<Pos>> pos_vector(this->car_num);
	std::vector<Pos> area_center;	// Voronoi center

	//std::cout << "SIZE " << way.size() << std::endl;
	//std::cout << "SIZE " << this->car_num <<std::endl;
	
	for (std::vector<Pos>::iterator i = way.begin(); i != way.end(); i++) {
		//pos_vector[i->group].push_back(Pos(i->x, i->y));
		pos_vector[i->group].push_back(Pos(i->x, i->y));
	}
	for (int g = 0; g < this->car_num; g++) {
		int count = 0;
		area_center.push_back(Pos(0, 0));
		for (std::vector<Pos>::iterator i = this->wayPoints.begin(); i != this->wayPoints.end(); i++) {
			if (i->group == g) {
				count++;
				area_center[g].x += i->x;
				area_center[g].y += i->y;
			}
		}
		area_center[g].x /= count;
		area_center[g].y /= count;
	}
	//for (std::vector<Pos>::iterator k = area_center.begin(); k != area_center.end(); k++) {
	//	std::cout << "!! " << k->x << " " << k->y << std::endl;
	//}
	int group = 0;
	for (std::vector<Pos*>::iterator i = this->ver.begin(); i != this->ver.end(); i++)
	{
		for (std::vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
		{
			if (((j->Left_Site.x == (*i)->x) && (j->Left_Site.y == (*i)->y)) || ((j->Right_Site.x == (*i)->x) && (j->Right_Site.y == (*i)->y)))
			{
				float min_dist = 10000;
				Pos AB;
				Pos AC;
				Pos newPose;

				//for (std::vector<Pos>::iterator k = area_center.begin(); k != area_center.end(); k++) {
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
		//std::cout << "POSE vector  "<<group << std::endl;
		for (std::vector<Pos>::iterator i = pos_vector[group].begin(); i != pos_vector[group].end(); i++) {
			//std::cout << "!! " << i->x << " " << i->y << std::endl;
		}
		group++;
	}
	group = 0;
	for (std::vector<std::vector<Pos>>::iterator i = pos_vector.begin(); i != pos_vector.end(); i++) {
		std::cout << "------------ Start" << std::endl;
		for (std::vector<Pos>::iterator a = i->begin(); a != i->end(); a++) {
			std::cout << a->x << " " << a->y << std::endl;
		}

		std::cout << "------------ SOrt" << std::endl;
		//QuickSort(*i, 1, i->size()-1);
		for (std::vector<Pos>::iterator a = i->begin(); a != i->end(); a++) {
			std::cout << a->x << " " << a->y << std::endl;
		}
		std::cout << "------------" << std::endl;
		for (std::vector<Pos>::iterator j = i->begin(); j != i->end(); j++) {
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
	for (std::vector<std::vector<VEdge>>::iterator i = edges_vector.begin(); i != edges_vector.end(); i++)
	{
		std::cout << "GROUP " << std::endl;

		for (std::vector<VEdge>::iterator j = i->begin(); j != i->end(); j++)
		{
			std::cout << "vector" << std::endl;
			std::cout << "		vector " << j->VertexA.x << " " << j->VertexA.y << std::endl;
			std::cout << "		vector " << j->VertexB.x << " " << j->VertexB.y << std::endl;
			//std::cout << "		vector "<<j->VertexA << " " << " " << j->VertexB << std::endl;
		}
	}

	return edges_vector;
}
//Temp
//void Engine::printWaypoints(string fileName) {
//	for (int i = 0; i < wayPoints.size(); i++) {
//		std::cout << "COORD = (" << wayPoints[i].x << " , " << wayPoints[i].y << ")  GROUP = " << wayPoints[i].group << std::endl;
//	}
//	std::ofstream writeFile;            //쓸 목적의 파일 선언
//	writeFile.open("text\\words"+fileName+".txt");
//
//
//	for (int i = 0; i < wayPoints.size(); i++) {
//		string str = to_string(int(wayPoints[i].x))+"\n";
//		writeFile.write(str.c_str(), str.size());
//		str = to_string(int(wayPoints[i].y)) + "\n";
//		writeFile.write(str.c_str(), str.size());
//		str = to_string(int(wayPoints[i].group)) + "\n";
//		writeFile.write(str.c_str(), str.size());
//	}
//}
// Unused
void Engine::getPath_voronoi() {
	KMean_clustering km(this->car_num, this->wayPoints.size());
	this->wayPoints = km.clustering(wayPoints);
	std::cout << std::endl << " ---------------------------------------------------------- " << std::endl;
	std::cout << "      KMeans Result " << std::endl << std::endl;
	//this->printWaypoints();

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
	for (std::vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
		std::cout << "(" << j->VertexA.x << ", " << j->VertexA.y << ")\t(" << j->VertexB.x << ", " << j->VertexB.y << ")\n";

	std::cout << " -------------------- " << std::endl;

	std::vector<std::vector<VEdge>> edges_vector;

	for (std::vector<Pos*>::iterator i = this->ver.begin(); i != this->ver.end(); i++)
	{
		std::vector<VEdge> edges;
		std::cout << "\t\tSite =  (" << (*i)->x << ", " << (*i)->y << ")\n";
		for (std::vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
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