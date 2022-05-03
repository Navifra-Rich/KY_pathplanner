#pragma once
#include"kmeans.cpp"
#include"voronoi.h"
#include<vector>
class Engine
{
private:
	int car_num = 0;
	vector<Pos*> ver;	// Voronoi center
	vector<Pos> ver_val;
	vector<VEdge> edges;
	//vector<VEdge> final_edges;
public:
	vector<Pos> wayPoints;
	void setCarnum(int car_num);
	void setWaypointRand(int waypoint_num);
	void getPath_voronoi();
	vector<vector<Pos>> getPath2();
	void printWaypoints();

	vector<vector<VEdge>> waypoints2vector(vector<Pos> way);
};

