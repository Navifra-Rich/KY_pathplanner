#include<iostream>
#include"Engine.h"

using namespace std;

int main() {
	Engine engine;

	//임의로 10개 waypoint 설정 (범위 = xy 0~100)
	//engine.setCarnum(3);
	//engine.setWaypointRand(10);
	//engine.getPath();
	//engine.waypoints2vector(engine.wayPoints);
	//engine.waypoints2vector(engine.area_center);

	engine.setCarnum(4);
	engine.setWaypointRand(10);
	engine.getPath2();


}
