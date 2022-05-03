#pragma once
#include"Pos.h"
#include<vector>
using namespace std;
class PathPlanner
{
private:
public:
	static vector<vector<Pos>> divide_intergral_center(vector<Pos>, double, int);
	static vector<vector<Pos>> divide_waypoints(const vector<Pos>);
	static vector<Pos> getInnerPoint_polygon(vector<Pos>, int width = 100, int height = 100);
	static void printWaypoints(vector<vector<Pos>>);
};

