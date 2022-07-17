#pragma once
#include"Pos.h"
#include<vector>
#include<cmath>
using namespace std;
class PathPlanner
{
private:
public:
	static std::vector<std::vector<Pos>> divide_intergral_center(std::vector<Pos>, double, int);
	static std::vector<std::vector<Pos>> divide_waypoints(const std::vector<Pos>);
	static std::vector<Pos> getInnerPoint_polygon(std::vector<Pos>, int width = 1000, int height = 1000);
	static void printWaypoints(std::vector<std::vector<Pos>>, std::vector<std::vector<float>>);
	static void printPathpoints(std::vector<Pos>);
	static std::vector<Pos> save_clustering_img(std::vector<Pos>);
	static void draw_Contours();
	static std::vector<std::vector<Pos>> samplingAllRoute(std::vector<std::vector<Pos>>); //Adjust interval of route
	static std::vector<Pos> samplingRoute(std::vector<Pos>, float); //Adjust interval of route
	static std::vector<float> getCurvature(std::vector<Pos> const& vecContourPoints);
	static std::vector<std::vector<float>> getCurvatureFromRoute(std::vector<std::vector<Pos>> route);
	//static void draw_Contours(int);
};

