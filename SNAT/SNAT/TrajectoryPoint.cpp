#include "stdafx.h"
#include "TrajectoryPoint.h"
#include <iostream> 
#include <sstream>

using namespace std;

TrajectoryPoint::TrajectoryPoint()
{
	this->lat = 0;
	this->lon = 0;
	this->rate_of_turn = 0;
	this->course_over_ground = 0;
	this->true_heading = 0;
	this->speed_over_ground = 0;
	this->mmsi = 0;
}

TrajectoryPoint::TrajectoryPoint(double lat, double lon)
{
	this->lat = lat;
	this->lon = lon;
	this->rate_of_turn = 0;
	this->course_over_ground = 0;
	this->true_heading = 0;
	this->speed_over_ground = 0;
	this->mmsi = 0;
}

TrajectoryPoint::TrajectoryPoint(double lat, double lon, double rate_of_turn,
	double course_over_ground, double true_heading, double speed_over_ground){
	this->lat = lat;
	this->lon = lon;
	this->rate_of_turn = rate_of_turn;
	this->course_over_ground = course_over_ground;
	this->true_heading = true_heading;
	this->speed_over_ground = speed_over_ground;
	this->mmsi = 0;
}

TrajectoryPoint::TrajectoryPoint(double lat, double lon, double rate_of_turn,
	double course_over_ground, double true_heading, double speed_over_ground, long mmsi){
	this->lat = lat;
	this->lon = lon;
	this->rate_of_turn = rate_of_turn;
	this->course_over_ground = course_over_ground;
	this->true_heading = true_heading;
	this->speed_over_ground = speed_over_ground;
	this->mmsi = mmsi;
}


TrajectoryPoint::~TrajectoryPoint()
{
}


double TrajectoryPoint::getLatitude() const {
	return this->lat;
}

void TrajectoryPoint::setLatitude(double lat) {
	this->lat = lat;
}

double TrajectoryPoint::getLongitude() const {
	return this->lon;
}

void TrajectoryPoint::setLongitude(double lon) {
	this->lon = lon;
}

double TrajectoryPoint::getRateOfTurn() const{
	return this->rate_of_turn;
}
void TrajectoryPoint::setRateOfTurn(double rate_of_turn){
	this->rate_of_turn = rate_of_turn;
}

double TrajectoryPoint::getCourseOverGround() const{
	return this->course_over_ground;
}
void TrajectoryPoint::setCourseOverGround(double course_over_ground){
	this->course_over_ground = course_over_ground;
}

double TrajectoryPoint::getTrueHeading() const{
	return this->true_heading;
}
void TrajectoryPoint::setTrueHeading(double true_heading){
	this->true_heading = true_heading;
}

double TrajectoryPoint::getSpeedOverGround() const{
	return this->speed_over_ground;
}
void TrajectoryPoint::setSpeedOverGround(double speed_over_ground){
	this->speed_over_ground = speed_over_ground;
}

long TrajectoryPoint::getMMSI() const{
	return this->mmsi;
}
void TrajectoryPoint::setMMSI(long mmsi){
	this->mmsi = mmsi;
}
bool TrajectoryPoint:: operator <(const TrajectoryPoint& rhs) const {
	// order by latitude when used in a set or map
	ostringstream my_ostream;
	string my_str, rhs_str;
	my_ostream << this->lat << "_"; 
	my_ostream << this->lon << "_";
	my_ostream << this->rate_of_turn << "_";
	my_ostream << this->course_over_ground << "_";
	my_ostream << this->true_heading << "_";
	my_ostream << this->speed_over_ground << "_";
	my_ostream << this->mmsi;
	my_str = my_ostream.str();
	my_ostream.str(std::string()); // clear content

	my_ostream << rhs.getLatitude() << "_";
	my_ostream << rhs.getLongitude() << "_";
	my_ostream << rhs.getRateOfTurn() << "_";
	my_ostream << rhs.getCourseOverGround() << "_";
	my_ostream << rhs.getTrueHeading() << "_";
	my_ostream << rhs.getSpeedOverGround() << "_";
	my_ostream << rhs.getMMSI();

	rhs_str = my_ostream.str();

	return my_str < rhs_str;
}

void TrajectoryPoint::print() {
	cout << "latitude:"<< this->lat << endl;
	cout << "longtitude"<<this->lon << endl;
	cout <<"rate_of_turn"<<this->rate_of_turn << endl;
	cout <<"course_over_ground"<< this->course_over_ground << endl;
	cout <<"true_heading"<< this->true_heading << endl;
	cout <<"speed_over_ground"<< this->speed_over_ground << endl;
	cout <<"mmsi"<< this->mmsi << endl;
}