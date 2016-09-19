#include "stdafx.h"
#include "PathMoverTrajectoryBased.h"
#include "TrajectoryLoader.h"
#include "Constants.h"
#include <fstream>
#include <sstream>
#include <float.h>
#include <time.h>
#include <assert.h>   


PathMoverTrajectoryBased::PathMoverTrajectoryBased()
{
	this->vessel_name_checked = map<string, bool>();
}


PathMoverTrajectoryBased::~PathMoverTrajectoryBased()
{
}



// wrapper methods and loading mechanism
void PathMoverTrajectoryBased::prepareOutFileStreams(string strAdd) {
	this->out_speed_f = new ofstream;
	this->out_course_f = new ofstream;
	this->own_target_history = new ofstream;
	this->out_target_history = new ofstream;

	std::string outFileNames;
	outFileNames = "PatternSpeed";
	outFileNames += strAdd;
	outFileNames += ".txt";
	//std::ofstream outSpeedF(outFileNames.c_str());
	(*this->out_speed_f).open(outFileNames.c_str());
	outFileNames = "PatternCourse";
	outFileNames += strAdd;
	outFileNames += ".txt";
	//std::ofstream outCourseF(outFileNames.c_str());
	(*this->out_course_f).open(outFileNames.c_str());
	outFileNames = "OwnHistory";
	outFileNames += strAdd;
	outFileNames += ".txt";
	//std::ofstream outOwnHistoryF(outFileNames.c_str());
	(*this->own_target_history).open(outFileNames.c_str());
	outFileNames = "TargetHistory";
	outFileNames += strAdd;
	outFileNames += ".txt";
	//std::ofstream outTargetHistoryF(outFileNames.c_str());
	(*this->out_target_history).open(outFileNames.c_str());
}

void PathMoverTrajectoryBased::loadPatternRelatedData() {

	TrajectoryLoader my_loader;
	time_t start_timer, end_timer;
	time(&start_timer);

	vector<Trajectory> all_patterns = my_loader.loadTrajectoryList(TRAJECTORY_INPUT_FOLDER_NAME + "/" +
		PROTOCOL_TRAJECTORIES_WITH_CLUSTER_SIZE_NAME);

	map<TrajectoryPoint, vector<Trajectory> > endpoints_to_protocol_mapping =
		my_loader.loadEndPointToTrajectoriesMap(TRAJECTORY_INPUT_FOLDER_NAME + "/" +
		ENDPOINTS_TO_PROTOCOL_TRAJECTORIES_NAME, all_patterns);

	vector <TrajectoryPoint> endpoints;
	for (map<TrajectoryPoint, vector<Trajectory> >::iterator it = endpoints_to_protocol_mapping.begin();
		it != endpoints_to_protocol_mapping.end(); ++it) {
		endpoints.push_back(it->first);
		//cout << "corresponding trajectory size:" << it->second.size() << endl;
	}
	cout << "number of endpoints read in:" << endpoints.size() << endl;

	map<string, VesselMinDistEntry> id_pair_str_to_min_dist = map<string, VesselMinDistEntry>();

	time(&end_timer);
	cout << "time spent for read trajectories:" << difftime(end_timer, start_timer) << endl;
	
	this->all_end_points = endpoints;
	this->endpoint_to_trajectory_map = endpoints_to_protocol_mapping;
	this->id_pair_str_to_min_dist = id_pair_str_to_min_dist;
	this->all_patterns = all_patterns;
}

void PathMoverTrajectoryBased::prepareRandomEngine() {
	std::default_random_engine generator(SEEDNUM); //the seed helps to repeat the experiment
	this->gen = generator;
}

void PathMoverTrajectoryBased::closeOutFileStreams() {
	(*this->out_course_f).close();
	(*this->out_speed_f).close();
	(*this->own_target_history).close();
	(*this->out_target_history).close();

}

void PathMoverTrajectoryBased::assignInitialPatternsToVessels(vector<Vessel> & vesselVector) {
	/*Assign the endpoints and corresponding trajetory to the vessels*/
	map<TrajectoryPoint, bool> endpoint_assigned = map<TrajectoryPoint, bool>();
	for (int i = 0; i < vesselVector.size(); i++) {
		cout << "try to assign for vessel" << i << endl;
		cout << "vessel vector size" << vesselVector.size() << endl;
		// for each vessel, pick a random endpoint and assign pattern based on cluster size of the protocol trajectories
		int selected_index = int(GeometryMethods::randGenerator(this->gen) * (this->all_end_points.size()));
		TrajectoryPoint new_end_point_to_start_with;
		// if the selected endpoint has no mapping/ mapping has no trajectory associated with it/ this endpoint already selected
		while (this->endpoint_to_trajectory_map.find(this->all_end_points[selected_index]) == this->endpoint_to_trajectory_map.end() ||
			this->endpoint_to_trajectory_map[this->all_end_points[selected_index]].size() == 0 ||
			endpoint_assigned.find(this->all_end_points[selected_index]) != endpoint_assigned.end()) {
			selected_index = int(GeometryMethods::randGenerator(this->gen) * (this->all_end_points.size())); // reselect
			//cout << "index generted this time :" << selected_index << endl;
		}
		new_end_point_to_start_with = this->all_end_points[selected_index];
		endpoint_assigned[this->all_end_points[selected_index]] = true;
		cout << "successfully assigned endpoint for vessel << " << i << endl;
	
		// choose new pattern with probabilty propotional to the size
		vector<Trajectory> potential_trajectories = this->endpoint_to_trajectory_map[new_end_point_to_start_with];
		cout << "this endpoint's assocaited patterns are:" << endl;
		for (int pattern_index = 0; pattern_index < potential_trajectories.size(); pattern_index++) {
			cout << potential_trajectories[pattern_index].getId() << " " << endl;
		}
		Trajectory final_new_pattern = PathMoverTrajectoryBased::selectTrajectoryAccordingToClusterSize(potential_trajectories,
			this->gen);

		// update v with final_new_pattern, also assign a new mmsi id
		vesselVector[i].initializeAttributesFromNewPattern(final_new_pattern);
		vesselVector[i].setMMSI(new_end_point_to_start_with.getMMSI());
	}
}

void PathMoverTrajectoryBased::assignPatternsToVesselsNearest(vector<Vessel> & vesselVector) {
	int count_failure = 0;
	for (int i = 0; i < vesselVector.size(); i++) {
		if (!this->alignVesselToNearestPattern(vesselVector[i], this->all_patterns)) {
			//cout << "failure to assign nearest pattern to vessel:" << vesselVector[i].getName() << endl;
			count_failure++;
		}
		else{
			//cout << "Success! to assign nearest pattern to vessel:" << vesselVector[i].getName() << endl;
		}
	}

	if (count_failure > 0) {
		cout << "Total number of failure in assigning nearest pattern:" << count_failure << endl;
		exit(-1); // exit program if there is a case where the pattern is not assigned, should never happen if initial positions have patterns close enought to them
	}
}

bool PathMoverTrajectoryBased::alignVesselToNearestPattern(Vessel &v, vector<Trajectory> &patterns) {
	TrajectoryPoint cur_tr_point = TrajectoryPoint(v.getLatitude(), v.getLongitude());
	vector<double> distance_dissimilarity;
	vector<double> course_dissimilarity;
	vector<int> nearest_positions;

	double this_distance, this_cosine_dissmilarity;
	int min_on_pattern_pos;
	pair<double, double> this_on_pattern_direction;
	pair<double, double> original_direction;
	double max_dist = -1;
	for (int i = 0; i < patterns.size(); i++) {
		this_distance = PathMoverTrajectoryBased::distanceFromTrajectory(patterns[i], cur_tr_point);
		// update max_dist
		if (this_distance > max_dist) {
			max_dist = this_distance;
		}
		min_on_pattern_pos = PathMoverTrajectoryBased::getNearestPatternPos(patterns[i], cur_tr_point);
		if (min_on_pattern_pos + 1 < patterns[i].getTrajectoryLength()) {
			this_on_pattern_direction = PathMoverTrajectoryBased::LatLonToXY(
				(*patterns[i].getPoints())[min_on_pattern_pos].getLatitude(),
				(*patterns[i].getPoints())[min_on_pattern_pos].getLongitude(),
				(*patterns[i].getPoints())[min_on_pattern_pos + 1].getLatitude(),
				(*patterns[i].getPoints())[min_on_pattern_pos + 1].getLongitude()
				);
		}
		else {
			this_on_pattern_direction = PathMoverTrajectoryBased::LatLonToXY(
				(*patterns[i].getPoints())[min_on_pattern_pos - 1].getLatitude(),
				(*patterns[i].getPoints())[min_on_pattern_pos - 1].getLongitude(),
				(*patterns[i].getPoints())[min_on_pattern_pos].getLatitude(),
				(*patterns[i].getPoints())[min_on_pattern_pos].getLongitude()
				);
		}

		original_direction = pair<double, double>(
			sin(GeometryMethods::toRadius(v.getCourse())),
			cos(GeometryMethods::toRadius(v.getCourse()))
			);
		// (1 - cosine theta) / 2 will give the cosine distance, from 0 to 1
		this_cosine_dissmilarity = (1.0 - PathMoverTrajectoryBased::cosineBetweenVectors(original_direction, this_on_pattern_direction) ) / 2.0;

		distance_dissimilarity.push_back(this_distance);
		course_dissimilarity.push_back(this_cosine_dissmilarity);
		nearest_positions.push_back(min_on_pattern_pos);
	}

	// normalized distance dissimilarity
	for (int i = 0; i < distance_dissimilarity.size(); i++) {
		distance_dissimilarity[i] = distance_dissimilarity[i] / max_dist;
		// cout << "distance_dissimilairty[" << i << "]" << distance_dissimilarity[i] << endl;
	}

	// find the pattern index with the lowest combined dissimilarity
	double min_dissimilarity = DBL_MAX;
	double pattern_index_to_assign;
	double this_combined_dissimilarity;
	for (int i = 0; i < patterns.size(); i++) {
		this_combined_dissimilarity = PathMoverTrajectoryBased::combinedDissimilarity(distance_dissimilarity[i], 
			course_dissimilarity[i]);
		if (this_combined_dissimilarity < min_dissimilarity) {
			min_dissimilarity = this_combined_dissimilarity;
			pattern_index_to_assign = i;
		}
	}

	double min_dist = distance_dissimilarity[pattern_index_to_assign];
	vector<TrajectoryPoint> pattern_points = (*patterns[pattern_index_to_assign].getPoints());
	if (min_dist < PATTERN_WIDTH + MIN_DISTANCE_MANEOUVERING) {
		// update the vessel attributes
		TrajectoryPoint first_point_on_pattern = (pattern_points)[0];
		v.setCourse(pattern_points[nearest_positions[pattern_index_to_assign]].getCourseOverGround());
		v.setSpeed(pattern_points[nearest_positions[pattern_index_to_assign]].getSpeedOverGround());
		v.setOnPatternPos(nearest_positions[pattern_index_to_assign]);
		v.setOrigin(first_point_on_pattern); // copy by value, a new object
		v.setPattern(patterns[pattern_index_to_assign]); // copy by value, a new object
		v.setMMSI(first_point_on_pattern.getMMSI());
		v.setOnPatternLat(pattern_points[nearest_positions[pattern_index_to_assign]].getLatitude());
		v.setOnPatternLon(pattern_points[nearest_positions[pattern_index_to_assign]].getLongitude());
		return true;
	}
	else {
		return false;
	}

}

void PathMoverTrajectoryBased::initialAlignmentForAllVesselsWrapper(vector<Vessel> &all_vessels) {
	this->initialAlignmentForAllVessels(
		all_vessels, 
		this->gen, 
		this->all_end_points, 
		this->endpoint_to_trajectory_map, 
		this->id_pair_str_to_min_dist);
}

void PathMoverTrajectoryBased::advanceAllOtherVesselsPatternBasedWrapper(double mt, vector<Vessel> &all_vessels, double runtime) {
	this->advanceAllOtherVesselsPatternBased(
		mt, 
		all_vessels, 
		this->gen, 
		this->all_end_points, 
		this->endpoint_to_trajectory_map, 
		this->id_pair_str_to_min_dist, 
		(*this->out_speed_f), 
		(*this->out_course_f), 
		runtime, 
		"NULL", 
		(*this->own_target_history), 
		(*this->out_target_history));
}

// end of wrapper


// auxiliary helping functions
pair<double, double> PathMoverTrajectoryBased:: XYToLatLon(double reference_lat, double reference_lon, double x, double y) {
	// x, y coordinate are such that rightwards is positive x axis and upwards is positive y axis
	double lat2 = reference_lat + y * 360 / 40000;
	double lon2 = reference_lon + x / (40000 * cos((reference_lat + lat2)* PI / 360) / 360);
	return pair<double,double>(lat2, lon2);
}

pair<double, double> PathMoverTrajectoryBased:: LatLonToXY(double lat1, double lon1, double lat2, double lon2) {
	double dx = (lon2 - lon1) * 40000 * cos((lat1 + lat2)* PI / 360) / 360; // approx using mid point of lat
	double dy = (lat2 - lat1) * 40000 / 360;
	return pair<double, double>(dx, dy);
}

double PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(TrajectoryPoint t1, TrajectoryPoint t2) {
	pair<double, double> dx_dy = PathMoverTrajectoryBased::LatLonToXY(t1.getLatitude(), t1.getLongitude(),
		t2.getLatitude(), t2.getLongitude());
	double dx = dx_dy.first;
	double dy = dx_dy.second;
	return sqrt(dx*dx + dy*dy);
}

double PathMoverTrajectoryBased::knotToKmPerMin(double knot) {
	return knot * 1.85200 / 60.0;
}

double PathMoverTrajectoryBased::adjustZeroSpeed(double knot) {
	if (knot <= 0.1) {
		return 0.1;
	}
	else {
		return knot;
	}
}

Trajectory PathMoverTrajectoryBased::selectTrajectoryAccordingToClusterSize(vector<Trajectory> potential_trajectories,
	default_random_engine & generator, int avoidId) {
	vector<int> rand_selection_sizes = vector<int>();
	for (int i = 0; i < potential_trajectories.size(); i++) {
		if (i == 0) {
			rand_selection_sizes.push_back(potential_trajectories[i].getClusterSize());
		}
		else {
			rand_selection_sizes.push_back(potential_trajectories[i].getClusterSize() + rand_selection_sizes[i - 1]);
		}
	}


	double rand_num_0_1 = GeometryMethods::randGenerator(generator);
	double rand_num = rand_num_0_1 *
		(rand_selection_sizes[rand_selection_sizes.size() - 1]);

	int index_to_use;
	for (int i = 0; i < rand_selection_sizes.size(); i++) {
		if (rand_num <= rand_selection_sizes[i]){
			index_to_use = i;
			break;
		}
	}

	return potential_trajectories[index_to_use];
}


Trajectory PathMoverTrajectoryBased::selectTrajectoryGivenDiscreteProbabilityVector(vector<Trajectory> potential_trajectories,
	vector<double> discrete_probs,
	default_random_engine & generator) {
	assert(potential_trajectories.size() == discrete_probs.size()); // tr length must correspond to its prob vector length
	vector<double> rand_selection_sizes = vector<double>();
	for (int i = 0; i < potential_trajectories.size(); i++) {
		if (i == 0) {
			rand_selection_sizes.push_back(discrete_probs[i]);
		}
		else {
			rand_selection_sizes.push_back(discrete_probs[i] + rand_selection_sizes[i - 1]);
		}
	}


	double rand_num_0_1 = GeometryMethods::randGenerator(generator);
	double rand_num = rand_num_0_1 *
		(rand_selection_sizes[rand_selection_sizes.size() - 1]);

	int index_to_use;
	for (int i = 0; i < rand_selection_sizes.size(); i++) {
		if (rand_num <= rand_selection_sizes[i]){
			index_to_use = i;
			break;
		}
	}

	return potential_trajectories[index_to_use];
}

double PathMoverTrajectoryBased::dotProduct(pair<double, double> vA, pair<double, double> vB) {
	return (vA.first * vB.first + vA.second * vB.second);
}

double PathMoverTrajectoryBased::vectorNorm(pair<double, double> v) {
	return sqrt(v.first * v.first + v.second * v.second);
}

double PathMoverTrajectoryBased::projectionAontoB(pair<double, double> vA, pair<double, double> vB) {
	double va_dot_vb = PathMoverTrajectoryBased::dotProduct(vA, vB);
	double vb_norm = PathMoverTrajectoryBased::vectorNorm(vB);
	return (va_dot_vb / vb_norm);
}

pair<double, double> PathMoverTrajectoryBased::projectionVectorAontoB(pair<double, double> vA, pair<double, double> vB) {
	double projection_va_on_vb = PathMoverTrajectoryBased::projectionAontoB(vA, vB);
	pair<double, double> projection_vector = pair<double, double>(
		projection_va_on_vb / PathMoverTrajectoryBased::vectorNorm(vB) * vB.first,
		projection_va_on_vb / PathMoverTrajectoryBased::vectorNorm(vB) * vB.second);
	return projection_vector;
}

pair<double, double> PathMoverTrajectoryBased::perpendicularVectorBtoA(pair<double, double> vA, pair<double, double> vB) {
	double projection_va_on_vb = PathMoverTrajectoryBased::projectionAontoB(vA, vB);
	pair<double, double> projection_vector = pair<double, double>(
		projection_va_on_vb / PathMoverTrajectoryBased::vectorNorm(vB) * vB.first,
		projection_va_on_vb / PathMoverTrajectoryBased::vectorNorm(vB) * vB.second);
	pair<double, double> perpendicular_v_from_vb_to_va = pair<double, double>(
		vA.first - projection_vector.first,
		vA.second - projection_vector.second);
	return perpendicular_v_from_vb_to_va;
}

vector<int> PathMoverTrajectoryBased::getAllPotentialPatternIdsFromEndpoints(
	vector<TrajectoryPoint> endpoints,
	map<TrajectoryPoint, vector<Trajectory> > endpoint_to_trajectory_map ) {
	vector<int> ids = vector<int>();
	for (int i = 0; i < endpoints.size(); i++) {
		if (endpoint_to_trajectory_map.find(endpoints[i]) != endpoint_to_trajectory_map.end()) {
			if (endpoint_to_trajectory_map[endpoints[i]].size() > 0) {
				for (int j = 0; j < endpoint_to_trajectory_map[endpoints[i]].size(); j++) {
					ids.push_back(endpoint_to_trajectory_map[endpoints[i]][j].getId());
				}
			}
		}
	}
	return ids;
}

bool PathMoverTrajectoryBased::isNeighbourVessel(Vessel v1, Vessel v2) {
	pair<double, double> dx_dy = PathMoverTrajectoryBased::LatLonToXY(
		v1.getLatitude(),
		v1.getLongitude(),
		v2.getLatitude(),
		v2.getLongitude()
		);

	return (sqrt(dx_dy.first * dx_dy.first + dx_dy.second * dx_dy.second) <= NEIGHBOURHOOD_SCAN_RADIUS);
}

bool PathMoverTrajectoryBased::targetPatternInPatterns(vector<Trajectory> &patterns, Trajectory &target_pattern) {
	for (int i = 0; i < patterns.size(); i++) {
		if (target_pattern.getId() == patterns[i].getId()) {
			return true;
		}
	}
	return false;
}

bool PathMoverTrajectoryBased::isTwoVectorParallel(pair<double, double> vA, pair<double, double> vB) {
	return ((vB.second * vA.first - vB.first * vA.second) == 0);
}
pair<double, double> PathMoverTrajectoryBased::getIntersection(
	pair<double, double> p1,
	pair<double, double> v1,
	pair<double, double> p2,
	pair<double, double> v2) {
	// p1, p2 are points, v1, v2 are their direction respectively, if not parallel
	double alpha = (p1.first * v1.second - p1.second * v1.first - p2.first * v1.second + p2.second * v1.first) / 
		(v2.first * v1.second - v2.second * v1.first);
	
	return pair<double, double>(p2.first + alpha * v2.first, p2.second + alpha * v2.second);
}

double PathMoverTrajectoryBased::cosineBetweenVectors(pair<double, double> v1, pair<double, double> v2) {
	if (PathMoverTrajectoryBased::dotProduct(v1, v2) == 0) {
		return 0;
	}
	else {
		return PathMoverTrajectoryBased::dotProduct(v1, v2) / (PathMoverTrajectoryBased::vectorNorm(v1) *
			PathMoverTrajectoryBased::vectorNorm(v2));
	}
}

double PathMoverTrajectoryBased::distanceFromTrajectory(Trajectory tr, TrajectoryPoint p) {
	double min_dist = DBL_MAX;
	double this_dist;
	for (int i = 0; i < (*tr.getPoints()).size(); i++) {
		this_dist = PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(
			(*tr.getPoints())[i],
			p);
		if (this_dist < min_dist) {
			min_dist = this_dist;
		}
	}
	return min_dist;
}

int PathMoverTrajectoryBased::getNearestPatternPos(Trajectory tr, TrajectoryPoint p) {
	double min_dist = DBL_MAX;
	double this_dist;
	int min_on_pattern_pos;
	for (int i = 0; i < (*tr.getPoints()).size(); i++) {
		this_dist = PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(
			(*tr.getPoints())[i],
			p);
		if (this_dist < min_dist) {
			min_dist = this_dist;
			min_on_pattern_pos = i;
		}
	}
	return min_on_pattern_pos;
}
// return true if p is far from each tr in the give vector of trajectories
bool PathMoverTrajectoryBased::farFromTrajectories(vector<Trajectory> trajectories, TrajectoryPoint p) {
	for (int i = 0; i < trajectories.size(); i++) {
		if (PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(p, (*trajectories[i].getPoints())[0]) < PATTERN_WIDTH) {
			return false;
		}
	}
	return true;
}

vector<Trajectory> PathMoverTrajectoryBased::filterAwayFarTrajectories(vector<Trajectory> trajectories, TrajectoryPoint p) {
	vector<Trajectory> close_trs = vector<Trajectory>();
	for (int i = 0; i < trajectories.size(); i++) {
		if (PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(p, (*trajectories[i].getPoints())[0]) < PATTERN_WIDTH) {
			close_trs.push_back(trajectories[i]);
		}
	}
	return close_trs;
}

double PathMoverTrajectoryBased::combinedDissimilarity(double distance_dissimilarity, double course_dissimilarity) {
	double DISTANCE_WEIGHT = 0.8;
	double COURSE_WEIGHT = 1.0 - DISTANCE_WEIGHT;
	return sqrt(DISTANCE_WEIGHT * distance_dissimilarity * distance_dissimilarity +
		COURSE_WEIGHT * course_dissimilarity * course_dissimilarity);
}
// end of auxiliary helping functions


// initial alignment function
void PathMoverTrajectoryBased::initialAlignmentForAllVessels(
	vector<Vessel> &all_vessels,
	std::default_random_engine & gen,
	vector<TrajectoryPoint> &all_end_points,
	map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
	map<string, VesselMinDistEntry> &id_pair_str_to_min_dist) {
	
	map<string, bool> visited = map<string, bool>();
	for (int i = 0; i < all_vessels.size(); i++) {
		visited[all_vessels[i].getName()] = true;
		this->arrangeVesselOnSamePattern(
			all_vessels[i],
			all_vessels,
			visited,
			id_pair_str_to_min_dist);
	}
}

void PathMoverTrajectoryBased::arrangeVesselOnSamePattern(Vessel &v, vector<Vessel> &all_vessels, map<string, bool> & visited,
	map<string, VesselMinDistEntry> &id_pair_str_to_min_dist) {
	
	Trajectory this_pattern = v.getPattern();
	vector<string> vessels_to_arrange = vector<string>();
	vessels_to_arrange.push_back(v.getName()); // assume I need to be aligned

	string this_vessel_name;
	for (int i = 0; i < all_vessels.size(); i++) {
		this_vessel_name = all_vessels[i].getName();
		if ((visited.find(this_vessel_name) == visited.end()) && // not visited
			this_vessel_name != v.getName() && // not same vessel
			all_vessels[i].getPattern().getId() == this_pattern.getId() && // same pattern
			PathMoverTrajectoryBased::isNeighbourVessel(v, all_vessels[i]) // is neighbour 
			) {
			vessels_to_arrange.push_back(all_vessels[i].getName());
		}
	}

	if (vessels_to_arrange.size() > 1) { // not just me
		int arrange_index = 0;
		for (int i = 0; i < all_vessels.size(); i++) {
			if (std::find(vessels_to_arrange.begin(), vessels_to_arrange.end(), all_vessels[i].getName()) != vessels_to_arrange.end()) { // need to be arranged
				TrajectoryPoint arrange_index_point = (*all_vessels[i].getPattern().getPoints())[arrange_index];
				all_vessels[i].setOnPatternPos(arrange_index);
				all_vessels[i].setLatitude(arrange_index_point.getLatitude());
				all_vessels[i].setLongitude(arrange_index_point.getLongitude());
				all_vessels[i].setSpeed(arrange_index_point.getSpeedOverGround());
				all_vessels[i].setCourse(arrange_index_point.getCourseOverGround());
				arrange_index ++;
				visited[all_vessels[i].getName()] = true;
			}
		}
	}
}


//main moving function
void PathMoverTrajectoryBased::advanceAllOtherVesselsPatternBased(double mt,
	vector<Vessel> &all_vessels,
	std::default_random_engine & gen,
	vector<TrajectoryPoint> &all_end_points,
	map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
	map<string, VesselMinDistEntry> &id_pair_str_to_min_dist,
	std::ofstream & outSpeedF, std::ofstream & outCourseF, double runtime, std::string avoidanceStr,
	std::ostream & OwnTargetHistory, std::ostream & outTargetHistory) {
	
	// check interaction, align, change course and speed and pattern 
	map<string, bool> visited = map<string, bool>();
	for (int i = 0; i < all_vessels.size(); i++) {
		visited[all_vessels[i].getName()] = true;
		this->alignVesselNeighbourhood(
			all_vessels[i],
			all_vessels,
			visited,
			id_pair_str_to_min_dist);
	}

	// just advance normally
	for (int i = 0; i < all_vessels.size(); i++)
	{
		this->advanceVesselPatternBased(
			mt,
			all_vessels[i],
			all_vessels,
			gen,
			all_end_points,
			endpoint_to_trajectory_map,
			id_pair_str_to_min_dist,
			outSpeedF,
			outCourseF,
			runtime,
			avoidanceStr,
			OwnTargetHistory,
			outTargetHistory);
	}

	// write pattern
	std::ofstream ofs;
	ofs.open("outPatternHistory.txt", std::ofstream::out | std::ofstream::app);

	std:ofstream ofs_on_pattern;
	ofs_on_pattern.open("outPatternLatLonHistory.txt", std::ofstream::out | std::ofstream::app);

	for (int i = 0; i < all_vessels.size(); i++) {
		all_vessels[i].writePatternToFile(runtime, ofs);
		all_vessels[i].writeOnPatternLatLonToFile(runtime, ofs_on_pattern);
	}
	ofs.close(); // TODO: avoid opening and closing for each loopTime tick
	ofs_on_pattern.close();

}

// clear append based file stream
void PathMoverTrajectoryBased::clearAppendBasedFileStream() {
	std::ofstream ofs;
	ofs.open("outPatternHistory.txt", std::ofstream::out);

	std:ofstream ofs_on_pattern;
	ofs_on_pattern.open("outPatternLatLonHistory.txt", std::ofstream::out);

	ofs.close(); // TODO: avoid opening and closing for each loopTime tick
	ofs_on_pattern.close();
}

// interaction based alignment
void PathMoverTrajectoryBased::alignVesselNeighbourhood(Vessel &v, vector<Vessel> &all_vessels, map<string, bool> & visited,
	map<string, VesselMinDistEntry> &id_pair_str_to_min_dist) {
	vector<int> pattern_ids = vector<int>();
	map<int, Trajectory> pattern_id_to_pattern = map<int, Trajectory>();
	map<int, int> pattern_ids_to_frequency = map<int, int>();
	vector<string> vessels_to_align = vector<string>();
	vessels_to_align.push_back(v.getName()); // assuem I need to aligned

	string this_vessel_name;
	for (int i = 0; i < all_vessels.size(); i++) {
		this_vessel_name = all_vessels[i].getName();
		if ((visited.find(this_vessel_name) == visited.end()) && // not visited
			this_vessel_name != v.getName() && // not same vessel
			PathMoverTrajectoryBased::isNeighbourVessel(v, all_vessels[i]) // is neighbour
			) {
			vessels_to_align.push_back(all_vessels[i].getName());
			pattern_ids.push_back(all_vessels[i].getPattern().getId());
			pattern_id_to_pattern[all_vessels[i].getPattern().getId()] = all_vessels[i].getPattern();
			if (pattern_ids_to_frequency.find(all_vessels[i].getPattern().getId()) == pattern_ids_to_frequency.end()) {
				pattern_ids_to_frequency[all_vessels[i].getPattern().getId()] = 1;
			}
			else {
				pattern_ids_to_frequency[all_vessels[i].getPattern().getId()] += 1;
			}

		}
	}

	if (pattern_ids_to_frequency.size() > 1) { // more than just my pattern, there is a pattern interaction
		// use a stochastic method to choose the pattern to follow, consider both the pattern size + number of vessels on patterns
		vector<Trajectory> potential_trs_to_follow = vector<Trajectory>();
		vector<double> discrete_probability_potential_trs = vector<double>();
		double FREQUENCY_WEIGHT = 0.8;
		double PATTERN_SIZE_WEIGHT = 0.2;
		for (map<int, Trajectory> ::iterator iter = pattern_id_to_pattern.begin();
			iter != pattern_id_to_pattern.end();
			iter++) {
			potential_trs_to_follow.push_back(iter->second);
			discrete_probability_potential_trs.push_back(FREQUENCY_WEIGHT * pattern_ids_to_frequency[iter->first] +
				PATTERN_SIZE_WEIGHT * iter->second.getClusterSize()); // prob = cluster size + current frequency of number of vessels
		}

		if (potential_trs_to_follow.size() != 0) {
			Trajectory pattern_to_follow = PathMoverTrajectoryBased::selectTrajectoryGivenDiscreteProbabilityVector(
				potential_trs_to_follow, discrete_probability_potential_trs, this->gen);
			//for (int tr_id = 0; tr_id < potential_trs_to_follow.size(); tr_id++) {
			//	cout << "potential pattesrn[" << tr_id << "]'s id = " << potential_trs_to_follow[tr_id].getId()
			//		<< ", size = " << potential_trs_to_follow[tr_id].getClusterSize() << endl;
			//}
			cout << "interaction pattern to follow:" << pattern_to_follow.getId() << endl;
			
			//write to file as well
			std::ofstream ofs;
			ofs.open("interationResult.txt", std::ofstream::out | std::ofstream::app);
			//for (int tr_id = 0; tr_id < potential_trs_to_follow.size(); tr_id++) {
			//	ofs << "potential pattern[" << tr_id << "]'s id = " << potential_trs_to_follow[tr_id].getId()
			//		<< ", size = " << potential_trs_to_follow[tr_id].getClusterSize() << endl;
			//}
			ofs << "interaction pattern to follow:" << pattern_to_follow.getId() << endl;
			ofs.close(); 

			// align all involved vessels to that max_freq_pattern_id
			for (int i = 0; i < all_vessels.size(); i++) {
				// if this vessel needs to aligned under this check
				if (std::find(vessels_to_align.begin(), vessels_to_align.end(), all_vessels[i].getName()) != vessels_to_align.end())
				{
					this->alignOneVesselToPattern(all_vessels[i], pattern_to_follow);
					visited[all_vessels[i].getName()] = true; // mark as visited, aligned
				}
			}
		}
	}
}

// single alignment of vessel to pattern
bool PathMoverTrajectoryBased::alignOneVesselToPattern(Vessel &v, Trajectory pattern) {
	vector<TrajectoryPoint> pattern_points = (*pattern.getPoints());
	int nearest_pos = 0; // TODO, back track nearest pos if the direction and position dot product is < 0
	double min_dist = DBL_MAX;
	double this_dist;
	for (int i = 0; i < pattern_points.size(); i++) {
		this_dist = PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(
			(*v.getPattern().getPoints())[v.getOnPatternPos()],
			pattern_points[i]);
		if (this_dist < min_dist) {
			min_dist = this_dist;
			nearest_pos = i;
		}
	}
	// Actually do the alignment if within pattern width + min maneuvering distance, otherwise, stay on original pattern
	if (min_dist < PATTERN_WIDTH + MIN_DISTANCE_MANEOUVERING) {
		// update the vessel attributes
		TrajectoryPoint first_point_on_pattern = (pattern_points)[0];
		v.setCourse(pattern_points[nearest_pos].getCourseOverGround());
		v.setSpeed(pattern_points[nearest_pos].getSpeedOverGround());
		v.setOnPatternPos(nearest_pos);
		v.setOrigin(first_point_on_pattern); // copy by value, a new object
		v.setPattern(pattern); // copy by value, a new object
		v.setMMSI(first_point_on_pattern.getMMSI());
		v.setOnPatternLat(pattern_points[nearest_pos].getLatitude());
		v.setOnPatternLon(pattern_points[nearest_pos].getLongitude());
		return true;
	}
	else {
		return false;
	}
}

// pattern based movement
void PathMoverTrajectoryBased::advanceVesselPatternBased(double mt, 
	Vessel &v, vector<Vessel> &all_vessels,
	std::default_random_engine & gen,
	vector<TrajectoryPoint> &all_end_points,
	map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
	map<string, VesselMinDistEntry> &id_pair_str_to_min_dist,
	std::ofstream & outSpeedF, std::ofstream & outCourseF, double runtime, std::string avoidanceStr,
	std::ostream & OwnTargetHistory, std::ostream & outTargetHistory)
{
	double old_speed, old_course; // actually, not really old, are the speed and course after alignment
	old_speed = v.getSpeed();
	old_course = v.getCourse();
	// check neighbourhood, align all vessels in detected neighbourhood (same course, same speed, same pattern, diff on pattern pos)

	// call the basic pattern based advancing functions
	this->advance(mt, v, all_vessels, all_end_points, endpoint_to_trajectory_map, id_pair_str_to_min_dist,
		gen);

	v.writeToOutStreams(old_speed, old_course, outSpeedF, outCourseF, runtime, avoidanceStr, OwnTargetHistory, outTargetHistory);
	return;
}


// advance vessel functions
void PathMoverTrajectoryBased::advance(double dt, Vessel &v, vector<Vessel> &all_vessels,
	vector<TrajectoryPoint> &all_end_points,
	map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
	map<string, VesselMinDistEntry> &id_pair_str_to_min_dist, 
	default_random_engine & generator) {
	// dt is in mins
	// basic move along the pattern of vessel v, checked before calling, so that here we just move forward
	int next_pos = v.getOnPatternPos() + 1; //check next point on pattern
	double move_speed;
	double this_pair_dist;
	double accumulated_t;
	pair<double, double> origin_on_pattern_point = pair<double, double>(
		v.getOnPatternLat(), v.getOnPatternLon());; // used in case 1 later


	if (next_pos >= (*v.getPattern().getPoints()).size()) { // if v.getOnPatternPos() is already trajectorylength - 1, might happen after alignment
		accumulated_t = 0;

		//pair<double, double> cur_pos_to_point = PathMoverTrajectoryBased::LatLonToXY(
		//	(*v.getPattern().getPoints())[next_pos - 2].getLatitude(),
		//	(*v.getPattern().getPoints())[next_pos - 2].getLongitude(), // next_pos -2 == v.getOnPatterPos() - 1
		//	v.getLatitude(),
		//	v.getLongitude());

		//pair<double, double> cur_pos_to_next_pos = PathMoverTrajectoryBased::LatLonToXY(
		//	(*v.getPattern().getPoints())[next_pos - 2].getLatitude(),
		//	(*v.getPattern().getPoints())[next_pos - 2].getLongitude(),
		//	(*v.getPattern().getPoints())[next_pos - 1].getLatitude(),
		//	(*v.getPattern().getPoints())[next_pos - 1].getLongitude());

		//double on_pattern_dist = PathMoverTrajectoryBased::projectionAontoB(cur_pos_to_point, cur_pos_to_next_pos);

		//pair <double, double> on_pattern_vector = PathMoverTrajectoryBased::projectionVectorAontoB(cur_pos_to_point, cur_pos_to_next_pos);

		//origin_on_pattern_point = PathMoverTrajectoryBased::XYToLatLon(
		//	(*v.getPattern().getPoints())[v.getOnPatternPos() - 1].getLatitude(),
		//	(*v.getPattern().getPoints())[v.getOnPatternPos() - 1].getLongitude(),
		//	on_pattern_vector.first,
		//	on_pattern_vector.second
		//	);

		//pair<double, double> perpendicular_pattern_to_point = PathMoverTrajectoryBased::perpendicularVectorBtoA(
		//	cur_pos_to_point,
		//	cur_pos_to_next_pos);

		//double to_pattern_dist = PathMoverTrajectoryBased::vectorNorm(perpendicular_pattern_to_point);

	}
	else { // not currently on the last point of pattern, cur_pos = next_pos - 1, -> on pattern index
		pair<double, double> cur_pos_to_next_pos = PathMoverTrajectoryBased::LatLonToXY(
			(*v.getPattern().getPoints())[next_pos - 1].getLatitude(),
			(*v.getPattern().getPoints())[next_pos - 1].getLongitude(),
			(*v.getPattern().getPoints())[next_pos].getLatitude(),
			(*v.getPattern().getPoints())[next_pos].getLongitude());

		pair<double, double> on_pattern_vector = PathMoverTrajectoryBased::LatLonToXY(
			(*v.getPattern().getPoints())[next_pos - 1].getLatitude(),
			(*v.getPattern().getPoints())[next_pos - 1].getLongitude(),
			v.getOnPatternLat(),
			v.getOnPatternLon());

		double on_pattern_dist = PathMoverTrajectoryBased::vectorNorm(on_pattern_vector);
		
		//pair<double, double> cur_pos_to_point = PathMoverTrajectoryBased::LatLonToXY(
		//	(*v.getPattern().getPoints())[next_pos - 1].getLatitude(),
		//	(*v.getPattern().getPoints())[next_pos - 1].getLongitude(),
		//	v.getLatitude(),
		//	v.getLongitude());

		//double on_pattern_dist = PathMoverTrajectoryBased::projectionAontoB(cur_pos_to_point, cur_pos_to_next_pos);

		//pair <double, double> on_pattern_vector = PathMoverTrajectoryBased::projectionVectorAontoB(cur_pos_to_point, cur_pos_to_next_pos);

		//origin_on_pattern_point = PathMoverTrajectoryBased::XYToLatLon(
		//	(*v.getPattern().getPoints())[next_pos - 1].getLatitude(),
		//	(*v.getPattern().getPoints())[next_pos - 1].getLongitude(),
		//	on_pattern_vector.first,
		//	on_pattern_vector.second
		//	);

		//pair<double, double> perpendicular_pattern_to_point = PathMoverTrajectoryBased::perpendicularVectorBtoA(
		//	cur_pos_to_point,
		//	cur_pos_to_next_pos);

		//to_pattern_dist = PathMoverTrajectoryBased::vectorNorm(perpendicular_pattern_to_point);
		//double cur_to_next_dist = PathMoverTrajectoryBased::vectorNorm(cur_pos_to_next_pos);
		
		move_speed = ((*v.getPattern().getPoints())[next_pos - 1].getSpeedOverGround() +
			(*v.getPattern().getPoints())[next_pos].getSpeedOverGround()) / 2.0;

		accumulated_t = (PathMoverTrajectoryBased::vectorNorm(cur_pos_to_next_pos) - on_pattern_dist) /
			PathMoverTrajectoryBased::knotToKmPerMin(PathMoverTrajectoryBased::adjustZeroSpeed(move_speed));

		// get the furthest on pattern pos that this move can go to within 1 min
		while (next_pos + 1 < v.getPattern().getTrajectoryLength() &&
			accumulated_t <= dt) {
			move_speed = PathMoverTrajectoryBased::adjustZeroSpeed((
				(*v.getPattern().getPoints())[next_pos].getSpeedOverGround() +
				(*v.getPattern().getPoints())[next_pos + 1].getSpeedOverGround()
				) / 2.0); // take the average speed
			this_pair_dist = PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint((*v.getPattern().getPoints())[next_pos],
				(*v.getPattern().getPoints())[next_pos + 1]);
			accumulated_t += this_pair_dist / PathMoverTrajectoryBased::knotToKmPerMin(move_speed); // in minutes
			next_pos += 1; // advance to next pos on pattern
		}
	}
	
	
	if (accumulated_t > dt && next_pos < v.getPattern().getTrajectoryLength() - 1) {
		// case 1, break the loop because the time to reach next pos is not enough
		// if still same pos, not time enough to advance far
		if (v.getOnPatternPos() + 1 == next_pos) {
			// no need to update speed, just update location and course, on pattern position is not changed,
			move_speed = (v.getSpeed() + (*v.getPattern().getPoints())[next_pos].getSpeedOverGround()) / 2.0; // use average speed!
			TrajectoryPoint cur_on_pattern_pos_point = (*v.getPattern().getPoints())[next_pos - 1];
			TrajectoryPoint target_point = (*v.getPattern().getPoints())[next_pos];
			pair<double, double> direction = PathMoverTrajectoryBased::LatLonToXY(
				cur_on_pattern_pos_point.getLatitude(),
				cur_on_pattern_pos_point.getLongitude(),
				target_point.getLatitude(),
				target_point.getLongitude());
			direction.first = direction.first / sqrt(direction.first * direction.first + direction.second * direction.second);
			direction.second = direction.second / sqrt(direction.first * direction.first + direction.second * direction.second);

			double dx = PathMoverTrajectoryBased::knotToKmPerMin(PathMoverTrajectoryBased::adjustZeroSpeed(move_speed))
				* direction.first *dt;
			double dy = PathMoverTrajectoryBased::knotToKmPerMin(PathMoverTrajectoryBased::adjustZeroSpeed(move_speed))
				* direction.second *dt;
			pair<double, double> new_lat_lon = PathMoverTrajectoryBased::XYToLatLon(v.getLatitude(), v.getLongitude(), dx, dy);
			v.setLatitude(new_lat_lon.first);
			v.setLongitude(new_lat_lon.second);
			
			pair<double, double> new_lat_lon_on_pattern = PathMoverTrajectoryBased::XYToLatLon(
				v.getOnPatternLat(),v.getOnPatternLon(), dx, dy);
			v.setOnPatternLat(new_lat_lon_on_pattern.first);
			v.setOnPatternLon(new_lat_lon_on_pattern.second);

			//update course
			double new_course = 90 - (atan2(direction.second, direction.first) * 180 / PI);
			if (new_course < 0) {
				new_course += 360;
			}
			v.setCourse(new_course);
		}
		else {
			vector<TrajectoryPoint> pattern_points = (*v.getPattern().getPoints());
			// need to update position, direction(course) and speed
			pair<double, double> direction = PathMoverTrajectoryBased::LatLonToXY(
				pattern_points[next_pos - 1].getLatitude(), pattern_points[next_pos - 1].getLongitude(), 
				pattern_points[next_pos].getLatitude(), pattern_points[next_pos].getLongitude());
			// normalize direction
			direction.first = direction.first / sqrt(direction.first * direction.first + direction.second * direction.second);
			direction.second = direction.second / sqrt(direction.first * direction.first + direction.second * direction.second);
			move_speed = PathMoverTrajectoryBased::knotToKmPerMin(PathMoverTrajectoryBased::adjustZeroSpeed((
				pattern_points[next_pos - 1].getSpeedOverGround() +
				pattern_points[next_pos].getSpeedOverGround()
				) / 2.0)); // take the average speed
			double time_between_nextpos_and_nextpos_prev = PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(
				pattern_points[next_pos - 1],
				pattern_points[next_pos]) / move_speed;
			double extra_t = dt - (accumulated_t - time_between_nextpos_and_nextpos_prev);
			
			// update lat, lon
			double dx = move_speed * direction.first * extra_t;
			double dy = move_speed * direction.second * extra_t;
			pair<double, double> new_on_pattern_lat_lon = PathMoverTrajectoryBased::XYToLatLon(
				pattern_points[next_pos -1].getLatitude(), 
				pattern_points[next_pos -1].getLongitude(), 
				dx, 
				dy);

			//pair <double, double> perpendicular_direction = pair<double, double>(-direction.second, direction.first);
			
			pair<double, double> on_pattern_origin_to_new = PathMoverTrajectoryBased::LatLonToXY(
				origin_on_pattern_point.first,
				origin_on_pattern_point.second,
				new_on_pattern_lat_lon.first,
				new_on_pattern_lat_lon.second
				);

			//pair<double, double> new_lat_lon_potential1 = PathMoverTrajectoryBased::XYToLatLon(
			//	new_on_pattern_lat_lon.first,
			//	new_on_pattern_lat_lon.second,
			//	perpendicular_direction.first * to_pattern_dist,
			//	perpendicular_direction.second * to_pattern_dist
			//	);
			//pair <double, double> new_lat_lon_potential2 = PathMoverTrajectoryBased::XYToLatLon(
			//	new_on_pattern_lat_lon.first,
			//	new_on_pattern_lat_lon.second,
			//	-perpendicular_direction.first * to_pattern_dist,
			//	-perpendicular_direction.second * to_pattern_dist
			//	);

			pair<double, double> new_lat_lon; // new lat lon
			
			new_lat_lon = PathMoverTrajectoryBased::XYToLatLon(v.getLatitude(), v.getLongitude(),
				on_pattern_origin_to_new.first,
				on_pattern_origin_to_new.second);

			//if (PathMoverTrajectoryBased::isTwoVectorParallel(on_pattern_origin_to_new, perpendicular_direction)) {
			//	// potentially two positions, choose the one that has the larger cos theta from (original on pattern pos to new on pattern pos) vector

			//	pair<double, double> origin_to_potential1 = PathMoverTrajectoryBased::LatLonToXY(
			//		v.getLatitude(),
			//		v.getLongitude(),
			//		new_lat_lon_potential1.first,
			//		new_lat_lon_potential1.second
			//		);
			//	pair<double, double> origin_to_potential2 = PathMoverTrajectoryBased::LatLonToXY(
			//		v.getLatitude(),
			//		v.getLongitude(),
			//		new_lat_lon_potential2.first,
			//		new_lat_lon_potential2.second
			//		);
			//	double cos_theta_1 = PathMoverTrajectoryBased::dotProduct(on_pattern_origin_to_new, origin_to_potential1) /
			//		(PathMoverTrajectoryBased::vectorNorm(on_pattern_origin_to_new) *
			//		PathMoverTrajectoryBased::vectorNorm(origin_to_potential1));

			//	double cos_theta_2 = PathMoverTrajectoryBased::dotProduct(on_pattern_origin_to_new, origin_to_potential2) /
			//		(PathMoverTrajectoryBased::vectorNorm(on_pattern_origin_to_new) *
			//		PathMoverTrajectoryBased::vectorNorm(origin_to_potential2));

			//	if (cos_theta_1 > cos_theta_2) {
			//		new_lat_lon = new_lat_lon_potential1;
			//	}
			//	else {
			//		new_lat_lon = new_lat_lon_potential2;
			//	}
			//}
			//else{
			//	// all relative to origin on pattern lat, lon
			//	pair<double, double> origin_pos_x_y = PathMoverTrajectoryBased::LatLonToXY(
			//		origin_on_pattern_point.first,
			//		origin_on_pattern_point.second,
			//		v.getLatitude(),
			//		v.getLongitude());
			//	pair<double, double> new_on_pattern_x_y = pair<double, double>(on_pattern_origin_to_new);

			//	pair<double, double> new_lat_lon_x_y = PathMoverTrajectoryBased::getIntersection(
			//		origin_pos_x_y, on_pattern_origin_to_new,
			//		new_on_pattern_x_y, perpendicular_direction);

			//	pair<double, double> new_lat_lon_geo = PathMoverTrajectoryBased::XYToLatLon(
			//		origin_on_pattern_point.first,
			//		origin_on_pattern_point.second,
			//		new_lat_lon_x_y.first,
			//		new_lat_lon_x_y.second);

			//	pair<double, double> true_perpendicular_direction = pair<double, double>(
			//		new_lat_lon_x_y.first - new_on_pattern_x_y.first,
			//		new_lat_lon_x_y.second - new_on_pattern_x_y.second
			//		);

			//	double cos_theta_1 = PathMoverTrajectoryBased::dotProduct(true_perpendicular_direction, perpendicular_direction) /
			//		(PathMoverTrajectoryBased::vectorNorm(true_perpendicular_direction) *
			//		PathMoverTrajectoryBased::vectorNorm(perpendicular_direction));

			//	pair<double, double> negative_perpendicular_direction(-perpendicular_direction.first, -perpendicular_direction.second);

			//	double cos_theta_2 = PathMoverTrajectoryBased::dotProduct(true_perpendicular_direction, negative_perpendicular_direction) /
			//		(PathMoverTrajectoryBased::vectorNorm(true_perpendicular_direction) *
			//		PathMoverTrajectoryBased::vectorNorm(negative_perpendicular_direction));

			//	if (cos_theta_1 > cos_theta_2) {
			//		new_lat_lon = new_lat_lon_potential1; // 1 is perpendicular direction
			//	}
			//	else {
			//		new_lat_lon = new_lat_lon_potential2; // 2 is - perpendicular direction
			//	}

			//}
			
			v.setLatitude(new_lat_lon.first);
			v.setLongitude(new_lat_lon.second);
			v.setOnPatternLat(new_on_pattern_lat_lon.first);
			v.setOnPatternLon(new_on_pattern_lat_lon.second);
			
			// update course
			double new_course = 90 - (atan2(direction.second, direction.first) * 180 / PI);
			if (new_course < 0) {
				new_course += 360;
			}
			v.setCourse(new_course);

			// update speed to prev point of next pos
			v.setSpeed(pattern_points[next_pos - 1].getSpeedOverGround());

			// update on pattern pos
			v.setOnPatternPos(next_pos - 1);
		}
	}
	else if (next_pos >= v.getPattern().getTrajectoryLength() - 1) { 
		// case 2: if reached the end of the pattern trajectory, assign new pattern and endpoint(if needed)
		
		// TODO: change to check new point based on the last point reached, not the last on pattern point
		TrajectoryPoint last_point_on_pattern = (*v.getPattern().getPoints())[v.getPattern().getTrajectoryLength() - 1];
		TrajectoryPoint cur_point_as_trajectory_point(v.getLatitude(), v.getLongitude());
		// check endpoints that start from this last_point_on_pattern
		vector<TrajectoryPoint> starting_endpoints = vector<TrajectoryPoint>();
		for (int i = 0; i < all_end_points.size(); i++){
			if (PathMoverTrajectoryBased::distanceBetweenTrajectoryPoint(cur_point_as_trajectory_point,
				all_end_points[i]) < NEIGHBOURHOOD_THRESH) {
				if (endpoint_to_trajectory_map.find(all_end_points[i]) != endpoint_to_trajectory_map.end()) {
					// strict condition on does not add endpoint that would possibly lead to the same pattern
					if (endpoint_to_trajectory_map[all_end_points[i]].size() > 0 &&
						!(PathMoverTrajectoryBased::targetPatternInPatterns(
						endpoint_to_trajectory_map[all_end_points[i]], 
						v.getPattern())) &&
						!(PathMoverTrajectoryBased::farFromTrajectories(
						endpoint_to_trajectory_map[all_end_points[i]], cur_point_as_trajectory_point))
						) {
						starting_endpoints.push_back(all_end_points[i]);
					}
				}
			}
		}

		TrajectoryPoint	new_end_point_to_start_with;
		if (starting_endpoints.size() > 0 ) {
			// randomly choose one from this starting_endpoints vector
			int selected_index = int(GeometryMethods::randGenerator(generator) * (starting_endpoints.size()));
			new_end_point_to_start_with = starting_endpoints[selected_index];

			// choose new pattern with probabilty propotional to the size
			vector<Trajectory> potential_trajectories = PathMoverTrajectoryBased::filterAwayFarTrajectories(
				endpoint_to_trajectory_map[new_end_point_to_start_with], cur_point_as_trajectory_point);
			Trajectory final_new_pattern = PathMoverTrajectoryBased::selectTrajectoryAccordingToClusterSize(potential_trajectories,
				generator);

			cout << "associated new pattern changed to: " << final_new_pattern.getId() << 
				"originally is : " << v.getPattern().getId() << endl;

			// update v with final_new_pattern, also assign a new mmsi id
			v.initializeAttributesFromNewPatternInteractionBased(final_new_pattern); // do not update origin, lat, lon, since starting from the original endpoint on previous pattern
			v.setMMSI(new_end_point_to_start_with.getMMSI());
		}
		else {
			// randomly choose one from all_endpoints that has at least one trajectory associated with it and
			// does not possibly lead to the same pattern again
			int selected_index = int(GeometryMethods::randGenerator(generator) * (all_end_points.size()));
			while (endpoint_to_trajectory_map.find(all_end_points[selected_index]) == endpoint_to_trajectory_map.end() ||
				endpoint_to_trajectory_map[all_end_points[selected_index]].size() == 0 ||
				PathMoverTrajectoryBased::targetPatternInPatterns(
				endpoint_to_trajectory_map[all_end_points[selected_index]],
				v.getPattern()
				)) {
				selected_index = int(GeometryMethods::randGenerator(generator) * (all_end_points.size())); // reselect
			}
			new_end_point_to_start_with = all_end_points[selected_index];

			// choose new pattern with probabilty propotional to the size
			vector<Trajectory> potential_trajectories = endpoint_to_trajectory_map[new_end_point_to_start_with];
			Trajectory final_new_pattern = PathMoverTrajectoryBased::selectTrajectoryAccordingToClusterSize(potential_trajectories,
				generator);

			cout << "random new pattern changed to: " << final_new_pattern.getId() << 
				"originally is : " << v.getPattern().getId() << endl;

			// update v with final_new_pattern, also assign a new mmsi id, update lat, lon, pattern origin as well
			v.initializeAttributesFromNewPattern(final_new_pattern);
			v.setMMSI(new_end_point_to_start_with.getMMSI());
		}

	}
	else {
		cout << "cases missed!" << endl;
	}

}



void PathMoverTrajectoryBased::changeCourse(double dt, Vessel &v){

}
void PathMoverTrajectoryBased::changeSpeed(double dt, Vessel &v){

}

// interaction based
void PathMoverTrajectoryBased::clearVesselNamesChecked() {
	this->vessel_name_checked.clear();
}
map<string, bool> PathMoverTrajectoryBased::getVesselNamesChecked() {
	return this->vessel_name_checked;
}