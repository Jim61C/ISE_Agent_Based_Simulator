#include "stdafx.h"
#include "UnitTester.h"
#include "Vessel.h"
#include "PathMoverTrajectoryBased.h"


UnitTester::UnitTester()
{
}


UnitTester::~UnitTester()
{
}


void UnitTester::unitTestLoadMMSIIds(TrajectoryLoader loader) {
	cout << TRAJECTORY_INPUT_FOLDER_NAME << endl;
	vector<long> ids = loader.loadVesselIDs(TRAJECTORY_INPUT_FOLDER_NAME + "/" + MMSI_LIST_NAME);
	for (int i = 0; i < ids.size(); i++) {
		cout << "id " << i << " loaded: " << ids[i] << endl;
	}
}

void UnitTester::unitTestLoadMINDistance(TrajectoryLoader loader) {
	map<string, VesselMinDistEntry> vessel_id_to_min_dist_entry = loader.loadMinDists(TRAJECTORY_INPUT_FOLDER_NAME +
		"/" + VESSEL_SPEED_TO_DISTANCE_NAME);
	cout << "number of pairs read in (x2)" << vessel_id_to_min_dist_entry.size() << endl;
	for (map<string, VesselMinDistEntry>::iterator itr = vessel_id_to_min_dist_entry.begin();
		itr != vessel_id_to_min_dist_entry.end(); itr++) {
		cout << "id pair: " << itr->first << ", size of speed_to_dist map:" << itr->second.getSpeedToMinDistMap()->size() << endl;
	}
}

void UnitTester::unitTestLoadEndpoints(TrajectoryLoader loader) {
	vector<Trajectory> trs = loader.loadTrajectoryList(TRAJECTORY_INPUT_FOLDER_NAME + "/" +
		PROTOCOL_TRAJECTORIES_WITH_CLUSTER_SIZE_NAME);

	map<TrajectoryPoint, vector<Trajectory> > endpoints_to_protocol_mapping =
		loader.loadEndPointToTrajectoriesMap(TRAJECTORY_INPUT_FOLDER_NAME + "/" +
		ENDPOINTS_TO_PROTOCOL_TRAJECTORIES_NAME, trs);

	vector <TrajectoryPoint> endpoints;
	for (map<TrajectoryPoint, vector<Trajectory> >::iterator it = endpoints_to_protocol_mapping.begin(); 
		it != endpoints_to_protocol_mapping.end(); ++it) {
		endpoints.push_back(it->first);
		//cout << "corresponding trajectory size:" << it->second.size() << endl;
	}
	cout << "number of endpoints read in:" << endpoints.size() << endl;
}

void UnitTester::unitTestLoadTrajectories(TrajectoryLoader loader) {
	vector<Trajectory> trs = loader.loadTrajectoryList(TRAJECTORY_INPUT_FOLDER_NAME + "/" +
		PROTOCOL_TRAJECTORIES_WITH_CLUSTER_SIZE_NAME);
	cout << "number of protocol trajecotries read in:" << trs.size() << endl;
}

void UnitTester::unitTestVesselAddingRemovingWaitingVesselNames() {
	Vessel temp("4352322", 200, 103.5, 1.34, 300, 14, 0);
	cout << "temp.addWaitingForVessel('12345')" << temp.addWaitingForVessel("12345") << endl;
	cout << "temp.addWaitingForVessel('33333')" << temp.addWaitingForVessel("33333") << endl;
	cout << "again, temp.addWaitingForVessel('12345')" << temp.addWaitingForVessel("12345") << endl;
	for (int i = 0; i < temp.getWaitingForVesselNames().size(); i++) {
		cout << temp.getWaitingForVesselNames()[i] << " " << endl;
	}

	cout << temp.removeWaitingForVessel("88888") << endl;
	cout << temp.removeWaitingForVessel("12345") << endl;
	cout << temp.removeWaitingForVessel("12345") << endl;

	for (int i = 0; i < temp.getWaitingForVesselNames().size(); i++) {
		cout << temp.getWaitingForVesselNames()[i] << " " << endl;
	}
}

void UnitTester::unitTestGetIntersection() {
	pair<double, double> temp1(1, 2);
	pair<double, double> temp2(temp1);
	cout << "temp2 right after copy from temp1:" << temp2.first << " , " << temp2.second << endl;
	temp1.first = 10;
	cout << "temp1 right after modify temp1:" << temp1.first << " , " << temp1.second << endl;
	cout << "temp2 right after modify temp1:" << temp2.first << " , " << temp2.second << endl;

	pair<double, double> v1(1, 2);
	pair<double, double> v2(2, 1);
	cout << "is v1, v2 parrallel?" << PathMoverTrajectoryBased::isTwoVectorParallel(v1, v2) << endl;
	pair<double, double> p1(1, 0);
	pair<double, double> p2(0, 1);

	pair<double, double> intersection1 = PathMoverTrajectoryBased::getIntersection(p1, v1, p2, v2);
	cout << "intersection1 : " << intersection1.first << ", " << intersection1.second << endl;
}

