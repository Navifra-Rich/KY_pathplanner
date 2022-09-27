#include "PathPlanner.h"
#include "hgMath.h"
#include<iostream>
#include <fstream>
#include<string>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

std::vector<Pos> PathPlanner::getInnerPoint_polygon(std::vector<Pos> wayPoints, int width, int height) {
	std::vector<Pos> interPoints;
	double omi = 0.0001;
	wayPoints.push_back(wayPoints[0]);
	for (int i = 0; i < width; i ++) {
		for (int j = 0; j < height; j ++) {
			int intersect_count = 0;
			Pos here(i + omi, j);
			for (std::vector<Pos>::iterator it = wayPoints.begin(); it < wayPoints.end() - 1; it++) {
				Pos there(i + omi, j + 90000);
				Pos intersect;
				bool isIntersected = hgMath::getIntersectPoint(*it, *(it + 1), here, there, &intersect);
				if (isIntersected) intersect_count++;
			}
			if (intersect_count % 2 == 1) interPoints.push_back(here);
		}
	}
	return interPoints;
}
std::vector<std::vector<Pos>> PathPlanner::divide_waypoints(const std::vector<Pos> wayPoints) {
	int max_group = 0;
	std::vector<std::vector<Pos>> temp(100);
	for (std::vector<Pos>::const_iterator i = wayPoints.begin(); i < wayPoints.end(); i++) {
		temp[i->group].push_back(*i);
		if (i->group > max_group) max_group = i->group;
	}
	std::vector<std::vector<Pos>> devided_wayPoints(temp.begin(), temp.begin() + max_group + 1);
	return devided_wayPoints;

}

std::vector<std::vector<Pos>> PathPlanner::divide_intergral_center(std::vector<Pos> wayPoints, double area_per_unit, int car_num) {
	std::vector<std::vector<Pos>> route;
	std::vector<Pos> mission_vec;
	Pos center = hgMath::getCenterPoint(wayPoints);
	wayPoints.push_back(wayPoints[0]);
	double cur_area = 0;		
	double cur_area_idx = 0;	
	
	// Start from center and first vertex
	mission_vec.push_back(center);
	mission_vec.push_back(wayPoints[0]);

	for (std::vector<Pos>::iterator i = wayPoints.begin(); i < wayPoints.end() - 1; i++) {
		Pos pivot = *i;
		Pos vec = *(i + 1) - *i;
		double dist = hgMath::getDist(vec);
		
		// Devide edge 1/1000 and search
		for (int sub_vec_idx = 0; sub_vec_idx < 1000; sub_vec_idx++) {
			Pos A, B;
			A = pivot + vec.mul(sub_vec_idx * 0.001);
			B = pivot + vec.mul((sub_vec_idx + 1) * 0.001);
			double area = hgMath::integral(center, A, B);
			cur_area += area;
			if ((area_per_unit < cur_area) &&
				(cur_area_idx + 1 != car_num)) {
				cur_area = 0;
				cur_area_idx++;
				mission_vec.push_back(B);
				route.push_back(mission_vec);
				mission_vec.clear();
				std::vector<Pos>().swap(mission_vec);
				mission_vec.push_back(center);
				mission_vec.push_back(B);
			}
		}
		mission_vec.push_back(*(i + 1));
	}
	route.push_back(mission_vec);
	return route;
}

std::vector<std::vector<Pos>> PathPlanner::samplingAllRoute(std::vector<std::vector<Pos>> route, float th, float deg) {
	std::vector<std::vector<Pos>> re_route;
	cout << "Route size " << route.size() << endl;
	for (int i = 0; i < route.size(); i++) {
		std::cout << "Pre  len " << route[i].size() << std::endl;
		re_route.push_back(samplingRoute(route[i], th, deg));
		std::cout << "Post len " << re_route[i].size() << std::endl;

	}
	return re_route;
}
std::vector<Pos> PathPlanner::samplingRoute(std::vector<Pos> points, float th, float deg) {
	std::vector<Pos> re_vector;
	re_vector.push_back(points[0]);
	int idx = 0;
	Pos pre_point;
	for (std::vector<Pos>::iterator it = (points.begin() + 1); it < points.end(); it++) {
		float dist = sqrt(pow(re_vector[idx].x - it->x, 2) + pow(re_vector[idx].y - it->y, 2));

		if (dist > th) {
			idx++;
			re_vector.push_back(Pos(it->x, it->y));
			continue;
		}
		if ((it + 1) < points.end()) {
			Pos vecA, vecB;
			vecA = Pos(re_vector[idx].x - it->x, re_vector[idx].y - it->y);
			vecB = Pos((it + 1)->x - it->x, (it + 1)->y - it->y);
			float innerV = vecA.x * vecB.x + vecA.y * vecB.y;
			float sA = sqrt(vecA.x * vecA.x + vecA.y * vecA.y);
			float sB = sqrt(vecB.x * vecB.x + vecB.y * vecB.y);
			float cosVec = acos(innerV / sA / sB) / 3.141592 * 180;

			if (abs(cosVec) < deg) {
				idx++;
				re_vector.push_back(Pos(it->x, it->y));
			}
		}
	}
	float last_dist = hgMath::getDist(re_vector[re_vector.size() - 1] - points[0]);
	if (last_dist > 10) re_vector.push_back(points[0]);
	return re_vector;
}
std::vector<Pos> PathPlanner::save_clustering_img(std::vector<Pos> points) {
	cv::Mat img_th, img_blur;
	cv::Mat img(1000, 1000, CV_8U, cv::Scalar(255));
	std::vector<std::vector<cv::Point>> contours;
	for (std::vector<Pos>::iterator it = points.begin(); it < points.end(); it++) {
		img.at<uchar>(it->x, it->y) = 0;
	}
	threshold(img, img_th, 80, 255, cv::THRESH_BINARY_INV);
	findContours(img_th, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);


	// Visualization
	cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
	cv::Scalar c(0, 0, 255);
	for (int i = 0; i < contours.size(); i++) {
		drawContours(img, contours, i, c, 3);
	}
	//cv::imshow("img", img);
	//cv::imshow("imgth", img_th);
	//cv::waitKey(0);

	std::vector<Pos> re_vector;
	for (std::vector<cv::Point>::iterator it = contours[0].begin(); it < contours[0].end(); it++) {
		Pos pos(it->y, it->x);
		re_vector.push_back(pos);
	}
	return re_vector;
}
std::vector<std::vector<Pos>> PathPlanner::getCarposeWithTimestamp(std::vector<std::vector<Pos>> route, std::vector<float> speed) {

	int car_num = route.size();
	std::vector<Pos> curCarPose;
	std::vector<bool> collision;
	std::vector<std::vector<Pos>> routeWithTime(car_num);
	std::vector<bool> routeFinished(car_num, false);
	std::vector<int> nextIndex;
	// car pose init
	for (int i = 0; i < car_num; i++) {
		routeWithTime[i].push_back(route[i][0]);
		curCarPose.push_back(route[i][0]);
		nextIndex.push_back(1);
	}

	while (true) {
		// ----------------- Car collision check -----------------------
		collision = checkCarDist(curCarPose);
		for (int i = 0; i < car_num; i++) {
			if (!collision[i] or routeFinished[i]) {
				routeWithTime[i].push_back(curCarPose[i]);
				continue;
			}
			float dist = speed[i];

			routeFinished[i] = moveCar(route[i], curCarPose[i], nextIndex[i], dist);
			routeWithTime[i].push_back(curCarPose[i]);
		}
		bool finished_flag = true;
		for (int i = 0; i < car_num; i++) if (!routeFinished[i]) finished_flag = false;
		if (finished_flag) return routeWithTime;
	}
	return routeWithTime;
}
std::vector<bool> PathPlanner::checkCarDist(std::vector<Pos> carPose, float th) {
	std::vector<bool> re_bool(carPose.size());
	for (int i = 0; i < carPose.size() - 1; i++) {
		for (int j = i; j < carPose.size(); j++) {
			float dist = (carPose[i].x - carPose[j].x) * (carPose[i].x - carPose[j].x) + (carPose[i].y - carPose[j].y) * (carPose[i].y - carPose[j].y);
			if (dist < th) re_bool[i] = false;
			else re_bool[i] = true;
		}
	}
	re_bool[carPose.size() - 1] = true;
	return re_bool;
}
bool PathPlanner::moveCar(std::vector<Pos> route, Pos& curPose, int& nextIdx, float& dist) {
	float remain_dist = dist;

	Pos vec = route[nextIdx] - curPose;
	float cur_path_dist = hgMath::getDist(vec);

	while (remain_dist > cur_path_dist) {
		remain_dist -= cur_path_dist;
		curPose = route[nextIdx];
		nextIdx++;
		if (nextIdx >= route.size()) {
			curPose = route[nextIdx - 1];
			return true;
		}
		vec = route[nextIdx] - curPose;
		cur_path_dist = hgMath::getDist(vec);
	}
	double ratio = remain_dist / cur_path_dist;
	Pos newCurPose(curPose.x + vec.x * ratio, curPose.y + vec.y * ratio);
	curPose = newCurPose;
	return false;
}
void PathPlanner::findNearestPoints(std::vector<vector<Pos>>& route, std::vector<Pos> raw_input) {
	float th = 25;
	for (int i = 0; i < route.size(); i++) {
		for (int ii = 0; ii < route[i].size(); ii++) {
			float min_dist = 10000;
			Pos near;
			for (int j = 0; j < raw_input.size(); j++) {
				Pos A = route[i][ii];
				Pos B = raw_input[j];
				Pos vec = A - B;
				float dist = hgMath::getDist(vec);
				if (dist < min_dist) {
					min_dist = dist;
					near = B;
				}
			}
			if (min_dist < th) route[i][ii] = near;
		}
	}
}





// ----------- Not used --------

std::vector<std::vector<float>> PathPlanner::getCurvatureFromRoute(std::vector<std::vector<Pos>> route) {
	std::vector<std::vector<float>> re_curvature;
	for (int i = 0; i < route.size(); i++) {
		vector<float> cva = getCurvature(route[i]);
		for (int ii = 0; ii < cva.size(); ii++) {
		}
		re_curvature.push_back(cva);
	}
	return re_curvature;
}
std::vector<float> PathPlanner::getCurvature(std::vector<Pos> const& vecContourPoints) {
	std::vector< float > vecCurvature;
	int idx = 0;
	//Pos pre_point;
	for (int i = 0; i < vecContourPoints.size(); i++) {
		int next_idx, pre_idx;

		Pos vecA, vecB;
		pre_idx = i - 1;
		next_idx = i + 1;
		if (pre_idx == -1)pre_idx = vecContourPoints.size() - 1;
		if (next_idx == vecContourPoints.size())next_idx = 0;
		vecA = Pos(vecContourPoints[i].x - vecContourPoints[pre_idx].x, vecContourPoints[i].y - vecContourPoints[pre_idx].y);
		vecB = Pos(vecContourPoints[next_idx].x - vecContourPoints[i].x, vecContourPoints[next_idx].y - vecContourPoints[i].y);
		//vecB = Pos((it + 1)->x - it->x, (it + 1)->y - it->y);
		float innerV = vecA.x * vecB.x + vecA.y * vecB.y;
		float sA = sqrt(vecA.x * vecA.x + vecA.y * vecA.y);
		float sB = sqrt(vecB.x * vecB.x + vecB.y * vecB.y);
		float cosVec = acos(innerV / (sA * sB));
		float degVec = cosVec / 3.141592 * 180;
		if (isnan(degVec)) {
			cout << "HEN!!" << endl;
			cout << "PRE  " << vecContourPoints[pre_idx].x << " " << vecContourPoints[pre_idx].y << endl;
			cout << "NOW  " << vecContourPoints[i].x << " " << vecContourPoints[i].y << endl;
			cout << "PST  " << vecContourPoints[next_idx].x << " " << vecContourPoints[next_idx].y << endl;
			cout << "Cos " << cosVec << endl;
			cout << "deg " << degVec << endl;
			if (std::abs(cosVec) >= 1) cout << "HEN!!" << endl;
			degVec = 0;
		}

		vecCurvature.push_back(degVec);
	}
	return vecCurvature;
}
void PathPlanner::printWaypoints(std::vector<std::vector<Pos>> wayPoints, std::vector<std::vector<float>> curvat) {
	int index = 0;
	for (std::vector<std::vector<Pos>> ::iterator i = wayPoints.begin(); i < wayPoints.end(); i++) {
		std::ofstream writeFile;           
		string fileName = to_string(index);
		writeFile.open("text\\words" + fileName + ".txt");
		int index2 = 0;
		for (std::vector<Pos> ::iterator j = i->begin(); j < i->end(); j++) {
			string str = to_string(int((*j).x)) + "\n";
			writeFile.write(str.c_str(), str.size());
			str = to_string(int((*j).y)) + "\n";
			writeFile.write(str.c_str(), str.size());
			index2++;
		}
		index++;
	}
}
void PathPlanner::printPathpoints(std::vector<Pos> wayPoints) {
	for (std::vector<Pos> ::iterator i = wayPoints.begin(); i < wayPoints.end(); i++) {
		std::cout << "		Point " << i->x << " " << i->y << std::endl;
	}
}
