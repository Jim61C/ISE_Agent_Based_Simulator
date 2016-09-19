#include "stdafx.h"
#include "VesselMinDistEntry.h"


VesselMinDistEntry::VesselMinDistEntry()
{
	this->id1 = 0; // 0 indicates does not exist this id 
	this->id2 = 0;
	this->speed_to_min_dist = map<double, double>();
}

VesselMinDistEntry::VesselMinDistEntry(long id1, long id2, map<double, double> speed_to_min_dist){
	this->id1 = id1;
	this->id2 = id2;
	this->speed_to_min_dist = speed_to_min_dist;
}


VesselMinDistEntry::~VesselMinDistEntry()
{
}


long VesselMinDistEntry::getId1(){
	return this->id1;
}
void VesselMinDistEntry::setId1(long id1){
	this->id1 = id1;
}

long VesselMinDistEntry::getId2(){
	return this->id2;
}
void VesselMinDistEntry::setId2(long id2){
	this->id2 = id2;
}

map<double, double> * VesselMinDistEntry::getSpeedToMinDistMap(){
	return &(this->speed_to_min_dist);
}
void VesselMinDistEntry::setSpeedToMinDistMap(map<double, double> speed_to_min_dist){
	this->speed_to_min_dist = speed_to_min_dist;
}