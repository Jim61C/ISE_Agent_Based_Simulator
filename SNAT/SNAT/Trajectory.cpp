#include "stdafx.h"
#include "Trajectory.h"


Trajectory::Trajectory()
{
	this->cluster_size = 1;
	this->id = -1;
}

Trajectory::Trajectory(vector<TrajectoryPoint> points, int cluster_size, int id)
{
	this->cluster_size = cluster_size;
	this->points = points;
	this->id = id;
}


Trajectory::~Trajectory()
{
}

vector<TrajectoryPoint>* Trajectory::getPoints(){
	return &(this->points);
}

int Trajectory::getTrajectoryLength(){
	return this->points.size();
}


void Trajectory::setPoints(vector<TrajectoryPoint> points){
	this->points = points;
}
int Trajectory::getClusterSize(){
	return this->cluster_size;
}
void Trajectory::setClusterSize(int cluster_size){
	this->cluster_size = cluster_size;
}

int Trajectory::getId() {
	return this->id;
}

void Trajectory::setId(int id) {
	this->id = id;
}