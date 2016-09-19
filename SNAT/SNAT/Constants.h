#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <string>

const long int SEEDNUM = 12345; // seed number used gloably

const string TRAJECTORY_INPUT_FOLDER_NAME = "trajectory_data";

const string ENDPOINTS_TO_PROTOCOL_TRAJECTORIES_NAME = "endpoints_to_protocol_trajectories.csv";
const string MMSI_LIST_NAME = "mmsi_list.csv";
const string PROTOCOL_TRAJECTORIES_WITH_CLUSTER_SIZE_NAME = "protocol_trajectories_with_cluster_size.csv";
const string VESSEL_MIN_DISTANCE_MATRIX_NAME = "vessel_min_distance_matrix.csv";
const string VESSEL_SPEED_TO_DISTANCE_NAME = "vessel_speed_to_distance.csv";
const double NEIGHBOURHOOD_THRESH = 0.5; // 0.5 km, used to determine the possible endpoints that starting from a certain location
const double NEIGHBOURHOOD_SCAN_RADIUS = 0.5; // for interaction behaviour
const double MIN_DISTANCE_MANEOUVERING = 0.5; // 500 metre safe distance is used if the extract value are too large

// for interaction, width of pattern
const double PATTERN_WIDTH = 0.5; // 0.5 on both sides, 1km width

#endif