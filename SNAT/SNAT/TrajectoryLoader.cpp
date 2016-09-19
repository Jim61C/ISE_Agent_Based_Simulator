#include "stdafx.h"
#include "TrajectoryLoader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>  
#include <stdlib.h> 


TrajectoryLoader::TrajectoryLoader()
{
}


TrajectoryLoader::~TrajectoryLoader()
{
}

vector<Trajectory> TrajectoryLoader::loadTrajectoryList(string infilename){
	vector<Trajectory> tr_vector;
	ifstream test_if(infilename);
	string nextline;
	string token;
	getline(test_if, nextline); // get rid of the headers
	bool cluster_size_set;
	int tr_id = 0; // sequence in loading is used as id
	while ( !test_if.eof()) {
		getline(test_if, nextline);
		cluster_size_set = false;
		int this_trajectory_cluster_size;
		vector<TrajectoryPoint> this_trajectory_points;
		while (nextline != "") {
			//cout << "new line:" << nextline << endl;
			double navigation_status, rate_of_turn, speed_over_ground,
				latitude, longitude, course_over_ground, true_heading;
			int cluster_size;
			ostringstream out;
			istringstream is(nextline);
			while (getline(is, token, ',')) {
				out<<token<<" ";
			}
			istringstream is_with_space(out.str());
			is_with_space >> navigation_status;
			is_with_space >> rate_of_turn;
			is_with_space >> speed_over_ground;
			is_with_space >> latitude;
			is_with_space >> longitude;
			is_with_space >> course_over_ground;
			is_with_space >> true_heading;
			is_with_space >> cluster_size;
			if (!cluster_size_set) {
				this_trajectory_cluster_size = cluster_size;
				cluster_size_set = true;
			}
			TrajectoryPoint this_trajecotory_point(latitude, longitude, rate_of_turn, 
				course_over_ground/10.0, true_heading, speed_over_ground); // course needs to be divided by 10
			this_trajectory_points.push_back(this_trajecotory_point);
			getline(test_if, nextline);
		}
		if (this_trajectory_points.size() > 0) { // avoid the case of a double new line break and nothing constructed for that trajectory
			cout << "id for pattern:" << tr_id << endl;
			tr_vector.push_back(Trajectory(this_trajectory_points, this_trajectory_cluster_size, tr_id));
			tr_id += 1; // increment id
		}
	}
	test_if.close();
	return tr_vector;
}
map<TrajectoryPoint, vector<Trajectory> > TrajectoryLoader::loadEndPointToTrajectoriesMap(string infilename, vector <Trajectory> &trajectory_list){
	map<TrajectoryPoint, vector<Trajectory> > endpoint_to_vector_of_trajetories;
	ifstream infile(infilename);
	string nextline;
	string token;
	getline(infile, nextline);// get rid of the header line
	while (!infile.eof()) {
		getline(infile, nextline);
		if (nextline != "") {
			double navigation_status, rate_of_turn, speed_over_ground,
				latitude, longitude, course_over_ground, true_heading, ts;
			long mmsi;
			string ts_string;

			vector<int> corresponding_trajetory_ids;
			vector<Trajectory> corresponding_trajectories;
			ostringstream out;
			istringstream is(nextline);
			ostringstream out_for_id_arr;
			while (getline(is, token, ',')) {
				// if found "[...]", indicating the start of the corresponding trajectory ids
				if (token.find("[") != -1){
					// put back to istringstream
					//cout << "arr token found:" << token << endl;
					for (int i = 0; i < token.length(); i++) {
						if (isdigit(token[i]) || token[i] == ' ') {
							out_for_id_arr << token[i];
						}
					}
					break;
				}
				else {
					out << token << " ";
				}
			}
			char temp;
			while (is.get(temp)) {
				if (isdigit(temp) || temp == ' ') {
					out_for_id_arr << temp;
				}
			}
			//cout << "id_arr_string (with / at two ends):" << "/" + out_for_id_arr.str() + "/" << endl;
			int tr_id_in_arr;
			istringstream tr_id_arr_stream(out_for_id_arr.str());
			while (tr_id_arr_stream >> tr_id_in_arr) {
				//cout << "push id:" << tr_id_in_arr << endl;
				corresponding_trajetory_ids.push_back(tr_id_in_arr);
			}

			//cout << "out.str()" << out.str() << endl;

			istringstream is_with_space(out.str());
			is_with_space >> navigation_status;
			is_with_space >> rate_of_turn;
			is_with_space >> speed_over_ground;
			is_with_space >> latitude;
			is_with_space >> longitude;
			is_with_space >> course_over_ground;
			is_with_space >> true_heading;
			is_with_space >> ts;
			is_with_space >> ts_string;
			is_with_space >> mmsi;
			TrajectoryPoint this_end_point(latitude, longitude, rate_of_turn, 
				course_over_ground/10.0, true_heading, speed_over_ground, mmsi);// course over ground needs to be divided by 10
			Trajectory this_tr;
			for (int i = 0; i < corresponding_trajetory_ids.size(); i++) {
				for (int j = 0; j < (*trajectory_list[corresponding_trajetory_ids[i]].getPoints()).size(); j++) {
					(*trajectory_list[corresponding_trajetory_ids[i]].getPoints())[j].setMMSI(this_end_point.getMMSI());
				}
				this_tr = trajectory_list[corresponding_trajetory_ids[i]];
				corresponding_trajectories.push_back(this_tr);
			}
			if (endpoint_to_vector_of_trajetories.find(this_end_point) != 
				endpoint_to_vector_of_trajetories.end()) {
				cout << "duplicated endpoint!" << endl;
				this_end_point.print();
			}
			endpoint_to_vector_of_trajetories.insert (
				pair<TrajectoryPoint, vector<Trajectory> >(this_end_point, corresponding_trajectories)
				);
		}
	}
	infile.close();
	return endpoint_to_vector_of_trajetories;

}
vector <long> TrajectoryLoader::loadVesselIDs(string infilename) {
	vector<long> mmsi;
	// input is the .csv file with a list of mmsi names and one header
	ifstream infile(infilename);
	string nextline;
	getline(infile, nextline);// get rid of the header line
	while (!infile.eof()) {
		getline(infile, nextline);
		if (nextline != "") {
			mmsi.push_back(strtol(nextline.c_str(), NULL, 10));
		}
	}
	infile.close();
	return mmsi;
}

map<string, VesselMinDistEntry> TrajectoryLoader::loadMinDists(string infilename){
	map<string, VesselMinDistEntry> id_pair_to_min_dist_enty;
	ifstream test_if(infilename);
	string nextline;
	string token;
	getline(test_if, nextline); // get rid of the headers
	bool id_pair_set;
	while (!test_if.eof()) {
		getline(test_if, nextline); // get next dataline, id1, id2, relative_speed, min_dist
		id_pair_set = false;

		long this_id1, this_id2;
		map<double, double> this_speed_to_min_dist;
		
		while (nextline != "") {
			//cout << "new line:" << nextline << endl;
			double relative_speed, min_dist;
			long id1, id2;
			ostringstream out;
			istringstream is(nextline);
			while (getline(is, token, ',')) {
				out << token << " ";
			}
			istringstream is_with_space(out.str());
			is_with_space >> id1;
			is_with_space >> id2;
			is_with_space >> relative_speed;
			is_with_space >> min_dist;
			if (!id_pair_set) {
				this_id1 = id1;
				this_id2 = id2;
				id_pair_set = true;
			}
			map<double, double>::iterator itr;
			if ((itr =  this_speed_to_min_dist.find(relative_speed)) != this_speed_to_min_dist.end()) { 
				// if already have this relative speed, update only if dist is smaller
				if (min_dist < itr->second) {
					// update
					itr->second = min_dist;
				}
			}
			else { // otherwise, just insert
				this_speed_to_min_dist.insert(pair<double, double>(relative_speed, min_dist));
			}
			getline(test_if, nextline);
		}
		if (this_speed_to_min_dist.size() > 0) { // avoid the case of a double new line break and nothing constructed for that trajectory
			string id_pair_str;
			string reverse_id_pair_str;
			ostringstream id_os;
			id_os << this_id1;
			id_os << "_";
			id_os << this_id2;
			id_pair_str = id_os.str();
			id_os.str(std::string()); // reset ostringstream
			id_os << this_id2;
			id_os << "_";
			id_os << this_id1;
			reverse_id_pair_str = id_os.str();
			VesselMinDistEntry this_vessel_min_dist_entry(this_id1, this_id2, this_speed_to_min_dist);
			id_pair_to_min_dist_enty.insert(pair<string, VesselMinDistEntry>(id_pair_str, this_vessel_min_dist_entry));
			id_pair_to_min_dist_enty.insert(pair<string, VesselMinDistEntry>(reverse_id_pair_str, this_vessel_min_dist_entry));
		}
	}
	test_if.close();
	return id_pair_to_min_dist_enty;
}