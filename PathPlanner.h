#pragma once
#include"Pos.h"
#include<vector>
using namespace std;
class PathPlanner
{
private:
public:
	static std::vector<std::vector<Pos>> divide_intergral_center(std::vector<Pos>, double, int);
	static std::vector<std::vector<Pos>> divide_waypoints(const std::vector<Pos>);
	static std::vector<Pos> getInnerPoint_polygon(std::vector<Pos>, int width = 1000, int height = 1000);
	static void printWaypoints(std::vector<std::vector<Pos>>);
	static void printPathpoints(std::vector<Pos>);
};

