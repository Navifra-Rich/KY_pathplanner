#include "PathPlanner.h"
#include "hgMath.h"
#include<iostream>
#include <fstream>
#include<string>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
// std::vector<Pos> -> vector<vector<Pos> 분리 (그룹별로)
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
// 폴리곤 내부에 유니폼 샘플링
std::vector<Pos> PathPlanner::getInnerPoint_polygon(std::vector<Pos> wayPoints, int width, int height) {
	std::vector<Pos> interPoints;
	int delta = 1;
	double omi = 0.0001;
	wayPoints.push_back(wayPoints[0]);
	for (int i = 0; i < width; i += delta) {
		for (int j = 0; j < height; j += delta) {
			int intersect_count = 0;
			Pos here(i + omi, j);
			for (std::vector<Pos>::iterator it = wayPoints.begin(); it < wayPoints.end() - 1; it++) {
				Pos there(i + omi, j + 99999999);
				Pos intersect;

				bool isIntersected = hgMath::getIntersectPoint(*it, *(it + 1), here, there, &intersect);
				if (isIntersected) intersect_count++;
			}
			//if (intersect_count == 0) 

			if (intersect_count % 2 == 1) {
				//std::cout << "InterSected!! HERE!! " << here.x << " " << here.y << std::endl; 
				//<< "   INTER!! " << intersect.x << " " << intersect.y << std::endl;
				interPoints.push_back(here);
			}
			//else
				//std::cout << "NO!!!!!!!!!!!! HERE!! " << here.x << " " << here.y << std::endl;
		}
	}
	std::cout << "Inter Point " << interPoints.size() << std::endl;
	return interPoints;
}
// 적분 기반 웨이포인트 반환
std::vector<std::vector<Pos>> PathPlanner::divide_intergral_center(std::vector<Pos> wayPoints, double area_per_unit, int car_num) {
	std::vector<std::vector<Pos>> route;
	std::vector<Pos> mission_vec;
	Pos center = hgMath::getCenterPoint(wayPoints);
	wayPoints.push_back(wayPoints[0]);
	double cur_area = 0;		// 현재 넓이
	double cur_area_idx = 0;	// 현재 차량 번호
	double cur_vec_idx = 0;	// 현재 선분 번호
	mission_vec.push_back(center);
	mission_vec.push_back(wayPoints[0]);
	for (std::vector<Pos>::iterator i = wayPoints.begin(); i < wayPoints.end() - 1; i++) {
		Pos pivot = *i;
		Pos vec = *(i + 1) - *i;
		double dist = hgMath::getDist(vec);

		// 10개로 선분 나눠서 넓이 계산
		for (int sub_vec_idx = 0; sub_vec_idx < 1000; sub_vec_idx++) {
			Pos A, B;
			A = pivot + vec.mul(sub_vec_idx * 0.001);
			B = pivot + vec.mul((sub_vec_idx + 1) * 0.001);
			double area = hgMath::integral(center, A, B);
			//tt += area;
			cur_area += area;
			//(area_per_unit * (1 - area_th) < cur_area) && (cur_area < area_per_unit* (1 + area_th))
			if ((area_per_unit < cur_area) &&
				(cur_area_idx + 1 != car_num)) {
				//std::cout << "AREA CHANGFE !!" << std::endl;
				std::cout << B.x << "  " << B.y << std::endl;
				cur_area = 0;
				cur_area_idx++;
				mission_vec.push_back(B);
				route.push_back(mission_vec);
				//for (std::vector<Pos> ::iterator j = mission_vec.begin(); j < mission_vec.end(); j++) {
				//	std::cout << "		Point " << j->x << " " << j->y << std::endl;
				//}
				mission_vec.clear();
				std::vector<Pos>().swap(mission_vec);
				mission_vec.push_back(center);
				mission_vec.push_back(B);
			}
			//if (cur_area_idx + 1 == this->car_num) break;
		}
		//if (cur_area_idx + 1 == this->car_num) break;
		mission_vec.push_back(*(i + 1));
		cur_vec_idx++;
	}
	route.push_back(mission_vec);
	return route;
}
// Print all the path
void PathPlanner::printWaypoints(std::vector<std::vector<Pos>> wayPoints, std::vector<std::vector<float>> curvat) {
	int index = 0;
	for (std::vector<std::vector<Pos>> ::iterator i = wayPoints.begin(); i < wayPoints.end(); i++) {
		//std::cout << "ROUTE " << std::endl;
		std::ofstream writeFile;            //쓸 목적의 파일 선언
		string fileName = to_string(index);
		writeFile.open("text\\words" + fileName + ".txt");
		int index2 = 0;

		for (std::vector<Pos> ::iterator j = i->begin(); j < i->end(); j++) {
			//std::cout << "		Point " << j->x << " " << j->y << std::endl;
			string str = to_string(int((*j).x)) + "\n";
			writeFile.write(str.c_str(), str.size());
			str = to_string(int((*j).y)) + "\n";
			writeFile.write(str.c_str(), str.size());
			str = to_string(float(curvat[index][index2])) + "\n";
			writeFile.write(str.c_str(), str.size());
			index2++;
		}
		index++;
	}
}
// Print only one path
void PathPlanner::printPathpoints(std::vector<Pos> wayPoints) {
	for (std::vector<Pos> ::iterator i = wayPoints.begin(); i < wayPoints.end(); i++) {
		std::cout << "		Point " << i->x << " " << i->y << std::endl;
	}
}
std::vector<float> PathPlanner::getCurvature(std::vector<Pos> const& vecContourPoints){
	std::vector< float > vecCurvature;
	int idx = 0;
	//Pos pre_point;
	for (int i = 0; i < vecContourPoints.size(); i++){
		int next_idx, pre_idx;

		Pos vecA, vecB;
		pre_idx	 = i - 1;
		next_idx = i + 1;
		if (pre_idx == -1)pre_idx= vecContourPoints.size()-1;
		if(next_idx == vecContourPoints.size())next_idx = 0;
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
		//if (abs(cosVec) > 70) {
		//	idx++;
		//	re_vector.push_back(Pos(it->x, it->y));
		//}
		////cout << "COS = " << cosVec << endl;
		//
		//}
		//pre_point = Pos(it->x, it->y);
	}
	return vecCurvature;

	//Pos posOld, posOlder;
	//Pos f1stDerivative, f2ndDerivative;
	//for (size_t i = 0; i < vecContourPoints.size(); i++)
	//{
	//	const Pos& pos = vecContourPoints[i];
	//
	//	if (i == 0) { posOld = posOlder = pos; }
	//
	//	f1stDerivative.x = pos.x - posOld.x;
	//	f1stDerivative.y = pos.y - posOld.y;
	//	f2ndDerivative.x = -pos.x + 2.0f * posOld.x - posOlder.x;
	//	f2ndDerivative.y = -pos.y + 2.0f * posOld.y - posOlder.y;
	//
	//	float curvature2D = 0.0f;
	//	if (std::abs(f2ndDerivative.x) > 0.1 && std::abs(f2ndDerivative.y) > 0.1)
	//	{
	//		curvature2D = sqrt(std::abs(
	//			pow(f2ndDerivative.y * f1stDerivative.x - f2ndDerivative.x * f1stDerivative.y, 2.0f) /
	//			(pow(std::abs(f2ndDerivative.x) + std::abs(f2ndDerivative.y), 3.0)+0.000001)));
	//	}
	//	if (curvature2D > 0) {
	//		cout << "CURVAT " << curvature2D << endl;
	//		cout << pos.x << " " << pos.y << "   " << posOld.x << " " << posOld.y << "        " << posOlder.x << " "<<posOlder.y << endl;
	//	}
	//	vecCurvature[i] = curvature2D;
	//
	//	posOlder = posOld;
	//	posOld = pos;
	//}
	return vecCurvature;
}

std::vector<std::vector<float>> PathPlanner::getCurvatureFromRoute(std::vector<std::vector<Pos>> route) {
	std::vector<std::vector<float>> re_curvature;
	for (int i = 0; i < route.size(); i++) {
		vector<float> cva = getCurvature(route[i]);
		for (int ii = 0; ii < cva.size(); ii++) {
			//cout << cva[ii] << endl;
		}
		re_curvature.push_back(cva);
	}
	return re_curvature;
}
std::vector<std::vector<Pos>> PathPlanner::samplingAllRoute(std::vector<std::vector<Pos>> route) {
	std::vector<std::vector<Pos>> re_route;
	for (int i = 0; i < route.size(); i++) {
		std::cout << "Pre  len " << route[i].size() << std::endl;
		re_route.push_back(samplingRoute(route[i],20));
		std::cout << "Post len " << re_route[i].size() << std::endl;
		
	}
	return re_route;
}
std::vector<Pos> PathPlanner::samplingRoute(std::vector<Pos> points, float th) {
	std::vector<Pos> re_vector;
	re_vector.push_back(points[0]);
	int idx = 0;
	Pos pre_point;
	for (std::vector<Pos>::iterator it = (points.begin()+1); it < points.end(); it++) {
		float dist = sqrt(pow(re_vector[idx].x - it->x, 2) + pow(re_vector[idx].y - it->y, 2));
		//cout << "DIST " << dist << endl;

		if (dist > th) {
			idx++;
			re_vector.push_back(Pos(it->x, it->y));
			continue;
		}
		if ((it + 1) < points.end()) {
			Pos vecA, vecB;
			vecA = Pos(it->x - re_vector[idx].x, it->y - re_vector[idx].y);
			vecB = Pos((it+1)->x- it->x, (it+1)->y -it->y);
			float innerV = vecA.x * vecB.x+ vecA.y * vecB.y;
			float sA = sqrt(vecA.x * vecA.x + vecA.y * vecA.y);
			float sB = sqrt(vecB.x*vecB.x + vecB.y*vecB.y);
			float cosVec = acos(innerV / sA / sB )/ 3.141592*180;
			if (abs(cosVec) > 70) {
				idx++;
				re_vector.push_back(Pos(it->x, it->y));
			}
			//cout << "COS = " << cosVec << endl;

		}
		//pre_point = Pos(it->x, it->y);
		

	}
	return re_vector;
}
std::vector<Pos> PathPlanner::save_clustering_img(std::vector<Pos> points) {
	cv::Mat img, img_th, img_blur;
	std::vector<std::vector<cv::Point>> contours;
	img.create(1000, 1000, CV_8U);
	for (std::vector<Pos>::iterator it = points.begin(); it < points.end(); it++) {
			img.at<uchar>(it->x, it->y) = 0;
	}
	//medianBlur(img, img_blur, 3);
	threshold(img, img_th, 80, 255, cv::THRESH_BINARY_INV);
	findContours(img_th, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
	

	// Visualization
	cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
	cv::Scalar c(0, 0, 255);
	for (int i = 0; i < contours.size(); i++) {
		cout << "IDX " << i << " SIZE " << contours[0].size() << endl;
		drawContours(img, contours, i, c, 3);
	}
	cv::imshow("img", img);
	//cv::imshow("img_bl", img_blur);
	cv::imshow("imgth", img_th);
	cv::waitKey(0);

	std::vector<Pos> re_vector;
	for (std::vector<cv::Point>::iterator it = contours[0].begin(); it < contours[0].end(); it++) {
		Pos pos(it->y, it->x);
		re_vector.push_back(pos);
	}
	return re_vector;
}
void PathPlanner::draw_Contours() {
	//cout << "HELLOW " << endl;
	//cv::Mat img, img_g, crop, crop_th;
	//img = cv::imread("C:\\Users\\admin\\Desktop\\weekly\\data\\star.png", cv::IMREAD_GRAYSCALE);
	//crop = img(cv::Rect(50, 50, img.cols - 100, img.rows - 100) & cv::Rect(0, 0, img.cols, img.rows));
	//threshold(crop, crop_th, 220, 255, cv::THRESH_BINARY_INV);
	//std::vector<std::vector<cv::Point>> contours;
	//findContours(crop_th, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
	//
	//cv::Mat dst;
	//cv::cvtColor(crop, dst, cv::COLOR_GRAY2BGR);
	//cv::Scalar c(0, 0, 255);
	//
	//for (int i = 0; i < contours.size(); i++) {
	//	drawContours(dst, contours, i, c, 3);
	//}
	//cv::imshow("img", img);
	//cv::imshow("dst", dst);
	//cv::imshow("crop", crop_th);
	//cv::waitKey(0);
}
