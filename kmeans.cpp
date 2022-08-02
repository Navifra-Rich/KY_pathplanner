#include<iostream>
#include<iomanip>
#include<string>
#include<algorithm>
#include<vector>
#include"Pos.h"
using namespace std;
class KMean_clustering {
private:
public:
	const int K_COUNT;// = 2;
	const int DATA_COUNT;// = 5;
	KMean_clustering(int k, int d): K_COUNT(k), DATA_COUNT(d) {}
	std::vector< Pos > datas;
	Pos* k = new Pos[K_COUNT];

	std::vector< Pos > clustering(std::vector< Pos > waypoints) {
		std::cout << " --------------------------------------------------------- " << std::endl;
		std::cout << "Hyper parameter" << std::endl << std::endl;
		std::cout << "K = " << K_COUNT << std::endl << std::endl;
		std::cout << "Input Count = " << DATA_COUNT << std::endl << std::endl;
		Pos* center = new Pos[K_COUNT];
		double* count_Group = new double[K_COUNT];
		std::vector< double >* distance = new std::vector< double >[K_COUNT];
		std::cout<< std::endl;
		std::cout << " --------------------------------------------------------- " << std::endl;
		//random k, init
		for (int i = 0; i < K_COUNT; i++)
		{
			k[i] = waypoints[i];
			center[i].x = waypoints[i].x;
			center[i].y = waypoints[i].y;
			distance[i].resize(DATA_COUNT);
			
			std::cout << "Center "<<center[i].x<<" "<< center[i].y<< ""<<std::endl;
		}
		std::cout << "HERE!!!!!!!!!!!!" << std::endl;
		bool loop = true;
		while (loop) { 
			//when the k-Positions are all same with next Position.
			//center init
			for (int i = 0; i < K_COUNT; i++) {
				center[i].x = 0;
				center[i].y = 0;
				count_Group[i] = 0;
			}
			// distance
			for (int i = 0; i < waypoints.size(); i++) {
				for (int j = 0; j < K_COUNT; j++) {
					double tmp_distance = sqrt(pow(k[j].x - waypoints[i].x, 2) + pow(k[j].y - waypoints[i].y, 2));
					distance[j][i] = tmp_distance;
				}
			}
			//get center
			for (int i = 0; i < waypoints.size(); i++) {
				double min = distance[0][i];
				int min_j = 0;
				for (int j = 1; j < K_COUNT; j++) {
					if (min > distance[j][i]) {
						min = distance[j][i];
						min_j = j;
					}
				}
				center[min_j].x += waypoints[i].x;
				center[min_j].y += waypoints[i].y;
				count_Group[min_j]++;
			}
			for (int i = 0; i < K_COUNT; i++) {
				std::cout << "Count Group " << count_Group[i]<<std::endl;
			}

			//change K
			int same_count = 0;
			for (int i = 0; i < K_COUNT; i++) {
				if (count_Group[i] != 0) {
					//cout << "K " << k[i].x << " " << k[i].y << endl;
					//cout << "C " << (center[i].x / count_Group[i]) << " " << (center[i].y / count_Group[i]) << endl;
					//if ((center[i].x / count_Group[i]) == k[i].x && (center[i].y / count_Group[i] == k[i].y)) {
					if (int(center[i].x / count_Group[i]*100) == int(k[i].x*100) && int(center[i].y / count_Group[i]*100) == int(k[i].y*100)) {
						same_count++;
					}
					k[i].x = center[i].x / count_Group[i];
					k[i].y = center[i].y / count_Group[i];
				}
				cout << "SC " << same_count << " K " << K_COUNT << endl;
				if (same_count == K_COUNT) {
					loop = false;
				}
			}
		}//end of loop
		for (int i = 0; i < waypoints.size(); i++) {
			double min = distance[0][i];
			int min_j = 0;
			for (int j = 1; j < K_COUNT; j++) {
				if (min > distance[j][i]) {
					min = distance[j][i];
					min_j = j;
				}
			}
			waypoints[i].group = min_j;
		}
		return waypoints;
	}
};