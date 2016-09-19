#pragma once
#include "Trajectory.h"
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include "VesselMinDistEntry.h"

using namespace std;

class TrajectoryLoader
{
public:
	TrajectoryLoader();
	~TrajectoryLoader();
	vector<Trajectory> loadTrajectoryList(string infilename);
	map<TrajectoryPoint, vector<Trajectory> > loadEndPointToTrajectoriesMap(string infilename, vector <Trajectory> &trajectory_list); // pass the trajectory_list as reference so that MMSI could be set for the trajectory points
	vector <long> loadVesselIDs(string infilename); // load vessel ids of the same vessel type
	map<string, VesselMinDistEntry> loadMinDists(string infilename);

};

