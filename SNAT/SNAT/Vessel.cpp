#include "Vessel.h"
#include <iostream>
#include <string>
#include <fstream>
#include "PathMover.h"
#include "geometry.h"
#include "PathMoverTrajectoryBased.h"
//#include "PathMoverTrajectoryBased.h"

using namespace std;
using std::string;
using namespace PathMover;
using namespace GeometryMethods;
//ofstream vesselFile("Infor.txt", ios::app);

Vessel :: Vessel (string aName, double aLength, double aLongitude, double aLatitude, double aCourse, double aSpeed, bool aStatus)
{
  Vessel_Name = aName;
  Vessel_Length = aLength;
  Vessel_Longitude = aLongitude;
  Vessel_Latitude = aLatitude;
  Vessel_Course = aCourse;
  Vessel_Speed = aSpeed;
  Vessel_Status = aStatus;
  Mover* myMover = new Mover();
  this->is_changing_pattern = false;
  this->wating_for_vessels_names = vector<string>();
}

// getters and setters
string Vessel :: getName (){return Vessel_Name;}
double Vessel :: getLength () {return Vessel_Length;}
double Vessel :: getLongitude (){return Vessel_Longitude;}
double Vessel :: getLatitude (){return Vessel_Latitude;}
double Vessel :: getCourse (){return Vessel_Course;}
double Vessel :: getSpeed (){return Vessel_Speed;}
bool Vessel :: getStatus (){return Vessel_Status;}
void Vessel :: setLongitude (double newLongitude){Vessel_Longitude = newLongitude;}
void Vessel :: setLatitude (double newLatitude){Vessel_Latitude = newLatitude;}
void Vessel :: setCourse (double newCourse){Vessel_Course = newCourse;}
void Vessel :: setSpeed (double newSpeed){Vessel_Speed = newSpeed;}
void Vessel :: setStatus (bool newStatus){Vessel_Status = newStatus;} 

void Vessel::setOrigin(TrajectoryPoint origin) {
	this->origin = origin;
}

TrajectoryPoint Vessel:: getOrigin() {
	return this->origin;
}

void Vessel::setPattern(Trajectory tr) {
	this->pattern = tr;
}
Trajectory Vessel::getPattern() {
	return this->pattern;
}

void Vessel::setOnPatternPos(int pos) {
	this->on_pattern_pos = pos;
}
int Vessel::getOnPatternPos() {
	return this->on_pattern_pos;
}

void Vessel::setMMSI(long mmsi) {
	this->MMSI = mmsi;
}
long Vessel::getMMSI() {
	return this->MMSI;
}

void Vessel::setOnPatternLat(double lat) {
	this->on_pattern_lat = lat;
}
double Vessel::getOnPatternLat() {
	return this->on_pattern_lat;
}

void Vessel::setOnPatternLon(double lon) {
	this->on_pattern_lon = lon;
}
double Vessel::getOnPatternLon() {
	return this->on_pattern_lon;
}

// printing functions
void Vessel::printPatternRelated() {
	TrajectoryPoint origin; // the current endpoint that is associated with this vessel
	Trajectory pattern; // trajectory pattern to follow
	int on_pattern_pos; // the cur pos on the pattern
	long MMSI;

	cout << "MMSI:" << this->MMSI << endl;
	cout << "pattern id:" << this->pattern.getId() << endl;
	cout << "pattern size:" << this->pattern.getClusterSize() << endl;
	cout << "origin:" << this->origin.getLatitude() << ", " << this->origin.getLongitude() << endl;
	cout << "on pattern pos:" << this->on_pattern_pos << endl;
	cout << endl;
}

void Vessel :: printVessel ()
{
  cout << Vessel_Name << " " << Vessel_Length << "m " << Vessel_Longitude << " " << Vessel_Latitude << " " << Vessel_Course << " " << Vessel_Speed << " " << Vessel_Status << endl;     
}  

void Vessel::outVessel(std::ofstream & vesselFile)
{
  vesselFile << Vessel_Name << " " << Vessel_Length << "m " << Vessel_Longitude << " " << Vessel_Latitude << " " << Vessel_Course << " " << Vessel_Speed << endl;     
} 

void Vessel::writePatternToFile(double runtime, ofstream & outPatternHistoryF) {
	outPatternHistoryF
		<< " " << (this->getPattern().getId())
		<< " " << this->getOnPatternPos()
		<< "/" << this->getPattern().getTrajectoryLength()
		<< " " << (*this->getPattern().getPoints())[this->on_pattern_pos].getLongitude()
		<< " " << (*this->getPattern().getPoints())[this->on_pattern_pos].getLatitude()
		<< " " << (*this->getPattern().getPoints())[this->on_pattern_pos].getCourseOverGround()
		<< " " << runtime << std::endl;
}

void Vessel::writeOnPatternLatLonToFile(double runtime, std::ofstream &outPatternHistoryF) {
	outPatternHistoryF 
		<< " " << (this->getPattern().getId())
		<< " " << this->getOnPatternPos()
		<< "/" << this->getPattern().getTrajectoryLength()
		<< " " << this->getOnPatternLon()
		<< " " << this->getOnPatternLat()
		<< " " << (*this->getPattern().getPoints())[this->on_pattern_pos].getCourseOverGround()
		<< " " << runtime << std::endl;
}

// called after update vessel movement to log the required attributes
void Vessel:: writeToOutStreams(double old_speed, double old_course, std::ofstream & outSpeedF, std::ofstream & outCourseF,
	double runtime, std::string avoidanceStr,
	std::ostream & OwnTargetHistory, std::ostream & outTargetHistory) {
	// log speed
	double schange = this->getSpeed() - old_speed;
	double newSpeed = this->getSpeed();
	Randspeed aRandspeed(this->getName(), old_speed, schange, newSpeed);
	aRandspeed.outRandspeed(outSpeedF);
	this->setSpeed(newSpeed);
	
	// log course
	double cchange = this->getCourse() - old_course;
	double newCourse = this->getCourse();
	if (newCourse > 360)
		newCourse = double(int(newCourse) % 360);
	while (newCourse < 0) {
		newCourse = 360 + newCourse;
	}
	Randcourse aRandcourse(this->getName(), this->getCourse(), cchange, newCourse);
	aRandcourse.outRandcourse(outCourseF);
	this->setCourse(newCourse);

	double newLongitude = this->getLongitude();
	double newLatitude = this->getLatitude();
	this->setLongitude(newLongitude);
	this->setLatitude(newLatitude);
	if (strcmp(avoidanceStr.c_str(), "NO"))
	{
		if (this->isOwnVessel == 1)
		{
			//Save subsequent covered locations of the Own vessel in the Own Vessel History
			if (strcmp(avoidanceStr.c_str(), "NULL"))
			{
				OwnTargetHistory << avoidanceStr << " ";
			}
			OwnTargetHistory << " " << newLongitude << " " << newLatitude << " " << runtime << std::endl;
		}
		else
		{
			//Save subsequent covered locations by the target vessels in Target vessels History

			// course to log is decided by either actual course used, or on pattern course
			double on_pattern_course = (*this->getPattern().getPoints())[this->on_pattern_pos].getCourseOverGround();
			double actual_course;
			if (this->getOnPatternPos() + 2 < this->getPattern().getTrajectoryLength()) {
				// Compare the on pattern course with v.getOnPatternPos() to v.getOnPatternPos() + 2
				pair<double, double> direction = PathMoverTrajectoryBased::LatLonToXY(
					(*this->getPattern().getPoints())[this->getOnPatternPos()].getLatitude(),
					(*this->getPattern().getPoints())[this->getOnPatternPos()].getLongitude(),
					(*this->getPattern().getPoints())[this->getOnPatternPos() + 2].getLatitude(),
					(*this->getPattern().getPoints())[this->getOnPatternPos() + 2].getLongitude()
					);
				// normalize direction vector
				direction.first = direction.first / sqrt(direction.first * direction.first + direction.second * direction.second);
				direction.second = direction.second / sqrt(direction.first * direction.first + direction.second * direction.second);
				actual_course = 90 - (atan2(direction.second, direction.first) * 180 / PI);
				if (actual_course < 0) {
					actual_course += 360;
				}
			}
			else {
				actual_course = this->getCourse();
			}
			double course_to_log;
			if (this->getPattern().getId() == 123) { // force logging actual course for the pattern with not so good course learned
				course_to_log = actual_course;
			}
			else if (abs(on_pattern_course - actual_course) < 80) {
				course_to_log = on_pattern_course;
			}
			else {
				course_to_log = actual_course;
			}
			outTargetHistory
				<< " " << (this->getPattern().getId())
				<< " " << this->getOnPatternPos()
				<< "/" << this->getPattern().getTrajectoryLength()
				<< " " << newLongitude << " " <<
				newLatitude << " " << course_to_log << " " << runtime << std::endl;
		}
	}
}

// random movement
void Vessel::advanceRand(double mt, std::default_random_engine & gen,
	std::ofstream & outSpeedF, std::ofstream & outCourseF, double runtime, std::string avoidanceStr,
	std::ostream & OwnTargetHistory, std::ostream & outTargetHistory)
{
	double old_speed, old_course;
	old_speed = this->getSpeed();
	old_course = this->getCourse();

	double schange = myMover->randSpeedChange(gen);
	double newSpeed = this->getSpeed()+schange*this->getSpeed();
	this->setSpeed(newSpeed);

	double cchange = myMover->randCourseChange(gen);
	double newCourse = this->getCourse() + cchange;
	if (newCourse > 360)
		newCourse = newCourse - 360;
	if (newCourse < 0)
		newCourse = 360 + this->getCourse() + cchange;
	this->setCourse(newCourse);

	double x_coordinate = this->getSpeed()*1.852 / 60 * sin(toRadius(this->getCourse()))*mt;
	double y_coordinate = this->getSpeed()*1.852 / 60 * cos(toRadius(this->getCourse()))*mt;
	double newLongitude = this->getLongitude() + 180 / PI / cos(toRadius(this->getLatitude()))*x_coordinate / 6371;
	double newLatitude = this->getLatitude() + 180 / PI*y_coordinate / 6371;
	this->setLongitude(newLongitude);
	this->setLatitude(newLatitude);
		
	this->writeToOutStreams(old_speed, old_course, outSpeedF, outCourseF, runtime, avoidanceStr, OwnTargetHistory, outTargetHistory);
	return;
}

void Vessel::initializeAttributesFromNewPattern(Trajectory new_pattern) {
	TrajectoryPoint first_point_on_pattern = (*new_pattern.getPoints())[0];
	this->setCourse(first_point_on_pattern.getCourseOverGround());
	this->setLatitude(first_point_on_pattern.getLatitude());
	this->setLongitude(first_point_on_pattern.getLongitude());
	this->setOnPatternPos(0);
	this->setOrigin(first_point_on_pattern); // copy by value, a new object
	this->setPattern(new_pattern); // copy by value, a new object
	this->setSpeed(first_point_on_pattern.getSpeedOverGround());
	this->setOnPatternLat(first_point_on_pattern.getLatitude());
	this->setOnPatternLon(first_point_on_pattern.getLongitude());
}

void Vessel::initializeAttributesFromNewPatternInteractionBased(Trajectory new_pattern) {
	// do not update vessel actual lat, lon
	TrajectoryPoint first_point_on_pattern = (*new_pattern.getPoints())[0];
	this->setCourse(first_point_on_pattern.getCourseOverGround());
	this->setOnPatternPos(0);
	this->setPattern(new_pattern); // copy by value, a new object
	this->setSpeed(first_point_on_pattern.getSpeedOverGround());
	this->setOnPatternLat(first_point_on_pattern.getLatitude());
	this->setOnPatternLon(first_point_on_pattern.getLongitude());
}

// interactoin related

void Vessel::setIsChangingPattern(bool changing) {
	this->is_changing_pattern = changing;
}
bool Vessel::getIsChangingPattern() {
	return this->is_changing_pattern;
}

void Vessel::setWaitingForVesselNames(vector<string> vessel_names) {
	this->wating_for_vessels_names = vessel_names;
}
vector<string> Vessel::getWaitingForVesselNames() {
	// return a copy, by value
	return this->wating_for_vessels_names;
}
bool Vessel::addWaitingForVessel(string vessel_name) {
	int i;
	for (i = 0; i < this->wating_for_vessels_names.size(); i++) {
		if (this->wating_for_vessels_names[i] == vessel_name) {
			break;
		}
	}
	if (i == this->wating_for_vessels_names.size()) {
		this->wating_for_vessels_names.push_back(vessel_name);
		return true;
	}
	else {
		return false;
	}
}
bool Vessel::removeWaitingForVessel(string vessel_name) {
	int i;
	for (vector<string>::iterator iter = this->wating_for_vessels_names.begin();
		iter != this->wating_for_vessels_names.end(); iter++) {
		if ((*iter) == vessel_name) {
			this->wating_for_vessels_names.erase(iter);
			return true;
		}
	}
	return false;
}