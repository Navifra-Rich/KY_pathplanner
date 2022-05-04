#pragma once
#include"kmeans.cpp"
#include"voronoi.h"
#include<vector>
class Engine
{
private:
	int car_num = 0;
	std::vector<Pos*> ver;	// Voronoi center
	std::vector<Pos> ver_val;
	std::vector<VEdge> edges;
	//std::vector<VEdge> final_edges;
public:
	std::vector<Pos> wayPoints;
	void setCarnum(int car_num);
	void setWaypointRand(int waypoint_num);
	void getPath_voronoi();
	std::vector<std::vector<Pos>> getPath2();
	void printWaypoints(string);

	std::vector<std::vector<VEdge>> waypoints2vector(std::vector<Pos> way);
};

