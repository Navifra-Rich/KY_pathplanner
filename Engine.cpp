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

void Engine::getWaypoint(vector<Pos> waypoint_temp)
{
    this->wayPoints = waypoint_temp;
}
void Engine::setCarnum(int car_num) {
	this->car_num = car_num;
}
void Engine::setWaypointRand(int waypoint_num) {
	std::cout << "Way Points" << std::endl<<std::endl;
	//3��: (280, 475)  (150.096, 250)  (409.904, 250)
	//	4�� : (280, 475)  (130, 325)  (280, 175)  (430, 325)
	//	5�� : (280, 475)  (137.342, 371.353)  (191.832, 203.647)  (368.168, 203.647)  (422.658, 371.353)
	//	6�� : (280, 475)  (150.096, 400)  (150.096, 250)  (280, 175)  (409.904, 250)  (409.904, 400)
	// ��~����
	//this->wayPoints.push_back(Pos(200,0));
	//this->wayPoints.push_back(Pos(400,0));
	//this->wayPoints.push_back(Pos(500,200 ));
	//this->wayPoints.push_back(Pos(300,400 ));
	//this->wayPoints.push_back(Pos(100,200));

	this->wayPoints.push_back(Pos(269.823, 500));
	this->wayPoints.push_back(Pos(450, 312.809));
	this->wayPoints.push_back(Pos(438.129, 105.814));
	this->wayPoints.push_back(Pos(250.350, 100));
	this->wayPoints.push_back(Pos(50, 281.058));
	this->wayPoints.push_back(Pos(58.649, 484.560));


	// ��~����
	//this->wayPoints.push_back(Pos(120, 50));
	//this->wayPoints.push_back(Pos(280, 50));
	//this->wayPoints.push_back(Pos(340, 210));
	//this->wayPoints.push_back(Pos(280, 190));
	//this->wayPoints.push_back(Pos(180, 210));
	// 
	// �����
	//this->wayPoints.push_back(Pos(280, 503));
	//this->wayPoints.push_back(Pos(130, 353));
	//this->wayPoints.push_back(Pos(280, 203));
	//this->wayPoints.push_back(Pos(430, 353));
	// 
	// ��~�簢��
	//this->wayPoints.push_back(Pos(0, 0));
	//this->wayPoints.push_back(Pos(20, 0));
	//this->wayPoints.push_back(Pos(20, 20));
	//this->wayPoints.push_back(Pos(0, 20));

	// �槄�簢��
	//this->wayPoints.push_back(Pos(10, 10));
	//this->wayPoints.push_back(Pos(110, 10));
	//this->wayPoints.push_back(Pos(110, 600));
	//this->wayPoints.push_back(Pos(10, 600));

	// ����
	//this->wayPoints.push_back(Pos(100.7, 0));
	//this->wayPoints.push_back(Pos(200.7, 200));
	//this->wayPoints.push_back(Pos(300.7, 0));
	//this->wayPoints.push_back(Pos(270.7, 200));
	//this->wayPoints.push_back(Pos(400.7, 400));
	//this->wayPoints.push_back(Pos(250.7, 400));
	//this->wayPoints.push_back(Pos(200.7, 600));
	//this->wayPoints.push_back(Pos(150.7, 400));
	//this->wayPoints.push_back(Pos(0, 400));
	//this->wayPoints.push_back(Pos(130, 200));
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
	std::vector<Pos> inter_points = PathPlanner::getInnerPoint_polygon(wayPoints);


	// --------------------------- Principal Component Analasis -------------------------
	std::vector<Pos> pca = hgMath::PCA(inter_points);
	if (pca[0].x == 0) pca[0].x += 0.000001;
	if (pca[1].y == 0) pca[1].y += 0.000001;
	double ratio = pca[0].x / (pca[1].y);
	if (ratio < 1) ratio = 1 / (ratio);
	std::cout << "RATIO " << ratio << std::endl;



	// --------------------------- Get path -------------------------
	std::vector<std::vector<Pos>> route;
	std::vector<std::vector<Pos>> route_integral;
	double threshold = 50;
	if (ratio < threshold) {
		// --------------------------- Get path by Kmeans clustering -------------------------
		std::cout << "Kmeans mode" << std::endl;
		KMean_clustering km(this->car_num, inter_points.size());
		std::vector<std::vector<Pos>> bf_convex_route;
		bf_convex_route = PathPlanner::divide_waypoints(km.clustering(inter_points));

		for (std::vector<std::vector<Pos>>::iterator it = bf_convex_route.begin(); it < bf_convex_route.end(); it++) {
			std::vector<Pos> convex_route = PathPlanner::save_clustering_img(*it);
			route.push_back(convex_route);
		}
	}
	else{
		// --------------------------- Get path by Integral -------------------------
		std::cout << "Integral" << std::endl;
		double area = hgMath::integral_sum(this->wayPoints);
		double area_per_unit = area / this->car_num;
		double area_th = 0.2;
		route_integral = PathPlanner::divide_intergral_center(this->wayPoints, area_per_unit, this->car_num);
		std::vector<std::vector<float>> cur1 = PathPlanner::getCurvatureFromRoute(route);

	}
	// --------------------------- Post Process -------------------------
	std::vector<std::vector<float>> cur1;
	route = PathPlanner::samplingAllRoute(route, 500, 134);
	route = PathPlanner::samplingAllRoute(route, 500, 165);
	PathPlanner::findNearestPoints(route, wayPoints);
	vector<float> car_speed(car_num, 50);
	car_speed[0] = 10;
	car_speed[2] = 110;
	route = PathPlanner::getCarposeWithTimestamp(route, car_speed);
	PathPlanner::printWaypoints(route, cur1);
	return route;
}