#pragma once
#include <map>
#include "Trajectory.h"

class VesselMinDistEntry
{
private:
	long id1;
	long id2;
	map<double, double> speed_to_min_dist;
public:
	VesselMinDistEntry();
	VesselMinDistEntry(long id1, long id2, map<double, double> speed_to_min_dist);
	~VesselMinDistEntry();

	// getter and setter
	long getId1();
	void setId1(long id1);

	long getId2();
	void setId2(long id2);

	map<double, double> * getSpeedToMinDistMap();
	void setSpeedToMinDistMap(map<double, double> speed_to_min_dist);
};

