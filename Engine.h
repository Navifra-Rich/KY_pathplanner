#pragma once
#include"kmeans.cpp"
#include"voronoi.h"
#include<vector>
class Engine
{
private:
	int car_num = 0;
	vector<Pos> wayPoints;
	vector<Pos*> ver;
	vector<VEdge> edges;
public:
	void setCarnum(int car_num);
	void setWaypointRand(int waypoint_num);
	void getPath();

	void printWaypoints();
};

