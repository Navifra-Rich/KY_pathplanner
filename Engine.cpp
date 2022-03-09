#pragma once
#include "Engine.h"

void Engine::setCarnum(int car_num) {
	this->car_num = car_num;
}
void Engine::setWaypointRand(int waypoint_num) {
	cout << "Way Points" << endl<<endl;
	for (int i = 0; i < waypoint_num; i++) {
		Pos tmp;
		tmp.x = (double)(rand() % 100);
		tmp.y = (double)(rand() % 100);
		cout << "(" << tmp.x << "," << tmp.y << ")  ";
		this->wayPoints.push_back(tmp);
	}
	cout << endl;
}

void Engine::getPath() {
	KMean_clustering km(this->car_num, this->wayPoints.size());
	this->wayPoints = km.clustering(wayPoints);
	cout << endl << " ---------------------------------------------------------- " << endl;
	cout << "      KMeans Result " << endl << endl;
	this->printWaypoints();


	Voronoi* vdg;
	for (int i = 0; i < km.K_COUNT; i++) {
		this->ver.push_back(new Pos(km.k[i].x, km.k[i].y));
	}

	vdg = new Voronoi();
	double minY = 0;
	double maxY = 100;
	this->edges = vdg->ComputeVoronoiGraph(this->ver, minY, maxY);
	delete vdg;
	cout << endl << " --------------------------------------------------------- " << endl;
	cout << "      Voronoi Result " << endl << endl;
	for (vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
		cout << "(" << j->VertexA.x << ", " << j->VertexA.y << ")\t(" << j->VertexB.x << ", " << j->VertexB.y << ")\n";


	for (vector<Pos*>::iterator i = this->ver.begin(); i != this->ver.end(); i++)
	{
		cout << "\t\tSite =  (" << (*i)->x << ", " << (*i)->y << ")\n";
		for (vector<VEdge>::iterator j = this->edges.begin(); j != this->edges.end(); j++)
		{
			if (((j->Left_Site.x == (*i)->x) && (j->Left_Site.y == (*i)->y)) || ((j->Right_Site.x == (*i)->x) && (j->Right_Site.y == (*i)->y)))
				cout << "(" << j->VertexA.x << ", " << j->VertexA.y << ")\t(" << j->VertexB.x << ", " << j->VertexB.y << ")\n";
		}
		cout << "\n";
	}
	//return 0;
}
void Engine::printWaypoints() {
	for (int i = 0; i < wayPoints.size(); i++) {
		cout << "COORD = (" << wayPoints[i].x << " , " << wayPoints[i].y << ")  GROUP = " << wayPoints[i].group << endl;
	}
	/*for (int i = 0; i < K_COUNT; i++) {
		cout << "Group " << i << " = " << "(" << k[i].x << " , " << k[i].y << ")" << endl;
	}*/
}