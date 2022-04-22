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
	void getPath();
	vector<vector<Pos>> getPath2();
	void printWaypoints();



	double integral_sum(vector<Pos>);
	double getDist(Pos);
	double integral(Pos, Pos, Pos);
	double inner(Pos, Pos);
	Pos getMidPoint(VEdge edge);
	Pos getCenterPoint(vector<Pos>);


	vector<vector<Pos>> getInnerPoints(vector<Pos> points, vector<vector<VEdge>> edges_vector, Pos center_point);
	vector<vector<VEdge>> waypoints2vector(vector<Pos> way);
};

