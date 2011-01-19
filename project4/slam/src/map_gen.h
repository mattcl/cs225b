#ifndef _map_gen_h
#define _map_gen_h

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include "graph.h"
#include "math.h"

class MapGen {
	public
	MapGen();
	~MapGen();

	void updateMap(Graph &graph);

	private:
	nam_msgs::OccupancyGrid map;

};

#endif
