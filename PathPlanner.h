#pragma once
#include"Pos.h"
#include<vector>
#include<cmath>
using namespace std;
class PathPlanner
{
private:
public:
	// ----------------------------- Func for kmeans clustering --------------------------
	static std::vector<Pos> getInnerPoint_polygon(std::vector<Pos>, int width = 1000, int height = 1000);		// Get points in the polygon for Kmeans clustering
	static std::vector<std::vector<Pos>> divide_waypoints(const std::vector<Pos>);								// Input clustered points into path vector
	static std::vector<Pos> save_clustering_img(std::vector<Pos>);												// Get path by Kmeans clustering algorithm

	
	// ----------------------------- Func for Integral --------------------------
	static std::vector<std::vector<Pos>> divide_intergral_center(std::vector<Pos>, double, int);				// Get path by integral algorithm
	

	// ----------------------------- Func for Post process --------------------------::
	static std::vector<std::vector<Pos>> samplingAllRoute(std::vector<std::vector<Pos>>, float th, float deg);	// Sampling route (get vertex from routes)
	static std::vector<Pos> samplingRoute(std::vector<Pos>, float th, float deg);								// Sampling route (determining if a point removed using dist, deg)
	static void findNearestPoints(std::vector<vector<Pos>>& route, std::vector<Pos> raw_input);					// Adjust point (because init value is float but unit of image pixel is int)
	//static std::vector<std::vector<Pos>> getCarposeWithTimestamp(std::vector<std::vector<Pos>> route, float speed);	//Sampling with car speed
	static std::vector<std::vector<Pos>> getCarposeWithTimestamp(std::vector<std::vector<Pos>> route, vector<float>);	//Sampling with car speed
	static std::vector<bool> checkCarDist(std::vector<Pos> carPose, float th = 10);								// Check the dist between cars
	static bool moveCar(std::vector<Pos> route, Pos& curPose, int& nextIdx, float& dist);						// Move the car for simulate
	

	// ----------------------------- Not used --------------------------
	static std::vector<std::vector<float>> getCurvatureFromRoute(std::vector<std::vector<Pos>> route);			// Get curvature from route
	static std::vector<float> getCurvature(std::vector<Pos> const& vecContourPoints);
	static void draw_Contours();
	
	// ----------------------------- Print --------------------------
	static void printWaypoints(std::vector<std::vector<Pos>>, std::vector<std::vector<float>>);					// Print routes
	static void printPathpoints(std::vector<Pos>);
};

