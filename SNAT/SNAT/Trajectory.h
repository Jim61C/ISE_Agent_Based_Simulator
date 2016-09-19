#pragma once
#include "TrajectoryPoint.h"
#include <vector>

using namespace std;

class Trajectory
{
private:
	vector<TrajectoryPoint> points; // will instantiate immediately, no need for new
	int cluster_size;
	int id; // id is just the index when loaded
public:
	Trajectory(); // default constructor
	Trajectory(vector<TrajectoryPoint> points, int cluster_size, int id); // pass by value of the vector of trajectory points 
	vector<TrajectoryPoint>* getPoints();
	int getTrajectoryLength();
	void setPoints(vector<TrajectoryPoint> points);
	int getClusterSize();
	void setClusterSize(int cluster_size);
	int getId();
	void setId(int id);
	~Trajectory();
};

