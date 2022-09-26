#pragma once
#include"kmeans.cpp"
//#include"voronoi.h"
#include<vector>
class Engine
{
private:
	int car_num = 0;
public:
	std::vector<Pos> wayPoints;
    void getWaypoint(vector<Pos> waypoint_temp);
	void setCarnum(int car_num);					//Set cars number
	void setWaypointRand(int waypoint_num);			//Set waypoints by code (for test only)
	std::vector<std::vector<Pos>> getPath2();		//Get path
};

