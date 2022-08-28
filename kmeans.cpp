#include<iostream>
#include<iomanip>
#include<string>
#include<algorithm>
#include<vector>
#include"Pos.h"
#include <cmath>

using namespace std;




class KMean_clustering {
private:
public:
	const int K_COUNT;// = 2;
	const int DATA_COUNT;// = 5;
	KMean_clustering(int k, int d): K_COUNT(k), DATA_COUNT(d) {}
	std::vector< Pos > datas;
	Pos* k = new Pos[K_COUNT];

	void print() {


	}
	std::vector< Pos > clustering(std::vector< Pos > waypoints) {
		std::cout << " --------------------------------------------------------- " << std::endl;
//        for(int i=0; i<waypoints.size(); i++)
//            cout<<"data: " << waypoints[i].x <<" "<< waypoints[i].y <<endl;

//		std::cout << "     Hyper parameter" << std::endl << std::endl;
//		std::cout << "K = " << K_COUNT << std::endl << std::endl;
//		std::cout << "Input Count = " << DATA_COUNT << std::endl << std::endl;
		//Pos* k = new Pos[K_COUNT];
		Pos* center = new Pos[K_COUNT];
		double* count_Group = new double[K_COUNT];
		std::vector< double >* distance = new std::vector< double >[K_COUNT];
//        std::cout << "111"<< std::endl << std::endl;
		//for (int i = 0; i < DATA_COUNT; i++) {
		//	Pos tmp;
		//	tmp.x = (double)(rand() % 100);
		//	tmp.y = (double)(rand() % 100);
		//	std::cout << "(" << tmp.x << "," << tmp.y << ")  ";
		//	datas.push_back(tmp);
		//}
		std::cout<< std::endl;
		//std::cout << " --------------------------------------------------------- " << std::endl;
		//random k, init
		for (int i = 0; i < K_COUNT; i++)
		{
			k[i] = waypoints[i];
			center[i].x = waypoints[i].x;
			center[i].y = waypoints[i].y;
			distance[i].resize(DATA_COUNT);
		}
//        std::cout << "222"<< std::endl << std::endl;
		bool loop = true;
        int iter=0;
		while (loop) { //when the k-Positions are all same with next Position.
//            cout<<"Iter "<<iter<<endl;
            iter++;
			//std::cout<<"
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
//            for (int i = 0; i < K_COUNT; i++) {
//                            std::cout << "Count Group " << count_Group[i]<<std::endl;
//                        }
			//change K
			int same_count = 0;
			for (int i = 0; i < K_COUNT; i++) {
				if (count_Group[i] != 0) {
                    if (int(center[i].x / count_Group[i]*100) == int(k[i].x*100) && int(center[i].y / count_Group[i]*100) == int(k[i].y*100))
                        same_count++;
//					if ((center[i].x / count_Group[i]) == k[i].x && (center[i].y / count_Group[i] == k[i].y))

					k[i].x = center[i].x / count_Group[i];
					k[i].y = center[i].y / count_Group[i];
				}
//                cout<<"SC "<<same_count<<" "<<K_COUNT<<endl;
				if (same_count == K_COUNT) {
					loop = false;
				}
				//std::cout << fixed << setprecision(2);
				//std::cout << "(" << k[i].x << "," << k[i].y << ") ";
			}
			//std::cout << std::endl;

		}//end of loop
        std::cout << "333"<< std::endl << std::endl;
		//std::cout << " --------------------------------------------------------- " << std::endl;
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
			//std::cout << i<<" "<<min_j << " "<<std::endl;
		}
		//std::cout << std::endl;
		return waypoints;

	}
};
