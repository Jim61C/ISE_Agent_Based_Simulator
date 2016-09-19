#ifndef PATHMOVERTRAJECTORYBASED_H
#define PATHMOVERTRAJECTORYBASED_H
#include "Trajectory.h"
#include "Vessel.h"
#include "VesselMinDistEntry.h"
#include <map>
#include <vector>

class PathMoverTrajectoryBased
{

private:
	map<string, bool> vessel_name_checked;
	// fields used when integrated,trajectory pattern data + ofstream + random generator
	std::default_random_engine gen;
	vector<TrajectoryPoint> all_end_points;
	map<TrajectoryPoint, vector<Trajectory> > endpoint_to_trajectory_map;
	map<string, VesselMinDistEntry> id_pair_str_to_min_dist;
	vector<Trajectory> all_patterns;
	std::ofstream * out_speed_f;
	std::ofstream * out_course_f;
	std::ofstream * own_target_history;
	std::ofstream * out_target_history;

public:
	PathMoverTrajectoryBased();
	~PathMoverTrajectoryBased();
	void advance(double dt, Vessel &v, vector<Vessel> &all_vessels, vector<TrajectoryPoint> &all_end_points,
		map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map, 
		map<string, VesselMinDistEntry> &id_pair_str_to_min_dist, 
		default_random_engine & generator); // advance the given vessel according to the parameter passed in
	void changeCourse(double dt, Vessel &v);
	void changeSpeed(double dt, Vessel &v);

	// loading of the needed data fields, all_end_points, endpoint_to_trajectory_map, id_pair_str_to_min_dist
	void prepareOutFileStreams(string strAdd); // needed to log out the information
	void loadPatternRelatedData(); // called at start
	void prepareRandomEngine(); // called at start
	void closeOutFileStreams(); // called at the end of the simulation
	// wrapper 
	void assignInitialPatternsToVessels(vector<Vessel> & vesselVector);
	/* assign the nearest pattern to the vessels given their current lat, lon and course; 
	possible that the some vessel might not get a pattern assigned in this way*/
	void assignPatternsToVesselsNearest(vector<Vessel> & vesselVector);
	bool alignVesselToNearestPattern(Vessel &v, vector<Trajectory> &patterns);
	void initialAlignmentForAllVesselsWrapper(vector<Vessel> &all_vessels);
	// mt is time step, eg, 1 min, runtime is the time step cout needed, 0,1,2,3...
	void advanceAllOtherVesselsPatternBasedWrapper(double mt, vector<Vessel> &all_vessels, double runtime);
	// end of wrapper

	// clear append based file stream
	void clearAppendBasedFileStream();

	// initial alignment function
	void initialAlignmentForAllVessels(
		vector<Vessel> &all_vessels,
		std::default_random_engine & gen,
		vector<TrajectoryPoint> &all_end_points,
		map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
		map<string, VesselMinDistEntry> &id_pair_str_to_min_dist);

	// main advance function
	void advanceAllOtherVesselsPatternBased(double mt,
		vector<Vessel> &all_vessels,
		std::default_random_engine & gen,
		vector<TrajectoryPoint> &all_end_points,
		map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
		map<string, VesselMinDistEntry> &id_pair_str_to_min_dist,
		std::ofstream & outSpeedF, std::ofstream & outCourseF, double runtime, std::string avoidanceStr,
		std::ostream & OwnTargetHistory, std::ostream & outTargetHistory);

	// advance function without interaction
	void PathMoverTrajectoryBased::advanceVesselPatternBased(double mt, 
		Vessel &v, vector<Vessel> &all_vessels, 
		std::default_random_engine & gen,
		vector<TrajectoryPoint> &all_end_points,
		map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
		map<string, VesselMinDistEntry> &id_pair_str_to_min_dist,
		std::ofstream & outSpeedF, std::ofstream & outCourseF, double runtime, std::string avoidanceStr = "NO",
		std::ostream & OwnTargetHistory = std::cout, std::ostream & outTargetHistory = std::cout);

	void assignEndpoint(Vessel &v, vector<TrajectoryPoint> &all_endpoints);
	vector<Vessel> detectNeighbourVessel(Vessel &v, vector<Vessel> &all_vessels); // will check this->vessel_names_checked 
	void alignVesselNeighbourhood(Vessel &v, vector<Vessel> &all_vessels, map<string, bool> & visited,
		map<string, VesselMinDistEntry> &id_pair_str_to_min_dist); // align v along with all its neighbours, using id_pair_str_to_min_dist
	bool alignOneVesselToPattern(Vessel &v, Trajectory pattern); // align v to the given pattern, find optimal on pattern pos, return true if actually aligned, false otherwise (since pattern has a fixed width)

	void arrangeVesselOnSamePattern(Vessel &v, vector<Vessel> &all_vessels, map<string, bool> & visited,
		map<string, VesselMinDistEntry> &id_pair_str_to_min_dist); // starting from this vessel, arrange vessels in its neighbour and on the same pattern sequentially 

	// auxiliary heping functions, static
	static pair<double, double> XYToLatLon(double reference_lat, double reference_lon, double x, double y);
	static pair<double, double> LatLonToXY(double lat1, double lon1, double lat2, double lon2);
	static double distanceBetweenTrajectoryPoint(TrajectoryPoint t1, TrajectoryPoint t2);
	static double knotToKmPerMin(double knot);
	static double adjustZeroSpeed(double knot);
	static Trajectory selectTrajectoryAccordingToClusterSize(vector<Trajectory> potential_trajectories,
		default_random_engine & generator, int avoidId = -1); // select a pattern out of a vector of patterns basd on their cluster size, the larget the cluster, the higher the probability
	static Trajectory selectTrajectoryGivenDiscreteProbabilityVector(vector<Trajectory> potential_trajectories,
		vector<double> discrete_probs,
		default_random_engine & generator);
	static double dotProduct(pair<double, double> vA, pair<double, double> vB);
	static double vectorNorm(pair<double, double> v);
	static double projectionAontoB(pair<double, double> vA, pair<double, double> vB);
	static pair<double, double> perpendicularVectorBtoA(pair<double, double> vA, pair<double, double> vB);
	static pair<double, double> projectionVectorAontoB(pair<double, double> vA, pair<double, double> vB);
	static vector<int> getAllPotentialPatternIdsFromEndpoints(
		vector<TrajectoryPoint> endpoints, map<TrajectoryPoint, vector<Trajectory> > endpoint_to_trajectory_map);
	static bool isNeighbourVessel(Vessel v1, Vessel v2);
	static bool targetPatternInPatterns(vector<Trajectory> &patterns, Trajectory &target_pattern);
	static bool isTwoVectorParallel(pair<double, double> vA, pair<double, double> vB);
	static pair<double, double> getIntersection(
		pair<double, double> p1, 
		pair<double, double> v1, 
		pair<double, double> p2, 
		pair<double, double> v2); // p1, p2 are points, v1, v2 are their direction respectively
	static double cosineBetweenVectors(pair<double, double> v1, pair<double, double> v2);
	static double distanceFromTrajectory(Trajectory tr, TrajectoryPoint p);
	static int getNearestPatternPos(Trajectory tr, TrajectoryPoint p);
	static bool farFromTrajectories(vector<Trajectory> trajectories, TrajectoryPoint p);
	static vector<Trajectory> filterAwayFarTrajectories(vector<Trajectory> trajectories, TrajectoryPoint p);
	static double combinedDissimilarity(double distance_dissimilarity, double course_dissimilarity);

	// interaction based
	void clearVesselNamesChecked();
	map<string, bool> getVesselNamesChecked();
};

#endif