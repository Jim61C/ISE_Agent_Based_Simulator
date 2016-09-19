#ifndef VESSEL_H
#define VESSEL_H
#include <string>
#include"PathMover.h"
#include "Trajectory.h"

using std::string;

class Vessel {

    private:
    string Vessel_Name; // the indentity key for vessel, unique
    double Vessel_Length;
    double Vessel_Longitude; 
    double Vessel_Latitude;
    double Vessel_Course;  
    double Vessel_Speed;
    bool Vessel_Status;
	PathMover::Mover* myMover;
	TrajectoryPoint origin; // the current endpoint that is associated with this vessel
	Trajectory pattern; // trajectory pattern to follow
	int on_pattern_pos; // the cur pos on the pattern
	long MMSI;
	// for interaction
	vector<string> wating_for_vessels_names;
	bool is_changing_pattern; // indicates that if the current vessel is currently shifting pattern
	double on_pattern_lat;
	double on_pattern_lon;
    
    public:
	bool isOwnVessel;
    Vessel (string, double, double, double, double, double,bool); //Constructor
    string getName ();
    double getLength ();
    double getLongitude ();
    double getLatitude ();
    double getCourse ();
    double getSpeed ();
    bool getStatus ();
    void setLongitude (double);
    void setLatitude (double);
    void setCourse (double);
    void setSpeed (double);
    void setStatus (bool);
	// printing functions
	void printPatternRelated();
    void printVessel ();
    void outVessel(std::ofstream & vesselF);
	// rand movement
	void advanceRand(double mt, std::default_random_engine & gen,
		std::ofstream & outSpeedF, std::ofstream & outCourseF, double runtime,std::string avoidanceStr = "NO",
		std::ostream & OwnTargetHistory = std::cout, std::ostream & outTargetHistory = std::cout);

	void advancePatternBased(double mt, std::default_random_engine & gen,
		std::ofstream & outSpeedF, std::ofstream & outCourseF, double runtime, std::string avoidanceStr = "NO",
		std::ostream & OwnTargetHistory = std::cout, std::ostream & outTargetHistory = std::cout);

	void setOrigin(TrajectoryPoint origin);
	TrajectoryPoint getOrigin();

	void setPattern(Trajectory tr);
	Trajectory getPattern();

	void setOnPatternPos(int pos);
	int getOnPatternPos();

	void setMMSI(long mmsi);
	long getMMSI();

	void setIsChangingPattern(bool changing);
	bool getIsChangingPattern();

	void setOnPatternLat(double lat);
	double getOnPatternLat();

	void setOnPatternLon(double lon);
	double getOnPatternLon();

	void setWaitingForVesselNames(vector<string> vessel_names);
	vector<string> getWaitingForVesselNames();
	bool addWaitingForVessel(string vessel_name);
	bool removeWaitingForVessel(string vessel_name);

	// initialize attributes according to first point on the new pattern
	void initializeAttributesFromNewPattern(Trajectory new_pattern);
	void initializeAttributesFromNewPatternInteractionBased(Trajectory new_pattern); // do not update origin, lat, lon

	// log the required info to file
	void writeToOutStreams(double old_speed, double old_course, std::ofstream & outSpeedF, std::ofstream & outCourseF,
		double runtime, std::string avoidanceStr,
		std::ostream & OwnTargetHistory, std::ostream & outTargetHistory);
	void writePatternToFile(double runtime, std::ofstream &outPatternHistoryF);
	void writeOnPatternLatLonToFile(double runtime, std::ofstream &outPatternHistoryF);
};
#endif