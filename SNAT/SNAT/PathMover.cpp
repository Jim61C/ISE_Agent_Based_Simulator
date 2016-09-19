#include "PathMover.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using std::string;

//ofstream collisionFile ("Collision.txt", ios::app);
using namespace PathMover;
using namespace GeometryMethods;

Manoeuvre::Manoeuvre()
{}

Manoeuvre::Manoeuvre(int aTime, double aCourse, double aSpeed)
{
	Time = aTime;
	Coursechange = aCourse;
	Speedchange = aSpeed;
}

int Manoeuvre::getTime(){ return Time; }
double Manoeuvre::getCourse() { return Coursechange; }
double Manoeuvre::getSpeed(){ return Speedchange; }

void Manoeuvre::printManoeuvre()
{
	cout << Time << " " << Coursechange << " " << Speedchange << endl;
}

void Manoeuvre::outManoeuvre(std::ofstream & manoeuvreFile)
{
	manoeuvreFile << Time << " " << Coursechange << " " << Speedchange << endl;
}

Randcourse::Randcourse(string aName, double aOriginal, double aChange, double aNew)
{
	Vessel_Name = aName;
	Original = aOriginal;
	Change = aChange;
	New = aNew;
}

string Randcourse::getName(){ return Vessel_Name; }
double Randcourse::getOriginal(){ return Original; }
double Randcourse::getChange() { return Change; }
double Randcourse::getNew(){ return New; }
void Randcourse::printRandcourse()
{
	cout << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}
void Randcourse::outRandcourse(std::ofstream & courseFile)
{
	courseFile << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}

Randspeed::Randspeed(string aName, double aOriginal, double aChange, double aNew)
{
	Vessel_Name = aName;
	Original = aOriginal;
	Change = aChange;
	New = aNew;
}

string Randspeed::getName(){ return Vessel_Name; }
double Randspeed::getOriginal(){ return Original; }
double Randspeed::getChange() { return Change; }
double Randspeed::getNew(){ return New; }
void Randspeed::printRandspeed()
{
	cout << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}
void Randspeed::outRandspeed(std::ofstream & speedFile)
{
	speedFile << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}

Mover::Mover()
{}
double Mover::randSpeedChange(std::default_random_engine & generator)
{
	double schange; //Percentage of change in speed
	if (randSpeed(generator))
	{

		if (randGenerator(generator) <= 0.27) //The change will be an increase in speed
		{
			schange = lognormGenerator(-0.99487, 0.77507, generator);
			return schange;
		}
		else //The change will be a decrease in speed
		{
			schange = -0.57881 + 0.19179*normGenerator(generator);
			return schange;
		}
	}
	else
		return 0.0;
}
double Mover::randCourseChange(std::default_random_engine & generator)
{
	double cchange; //Change in course in degree
	if (randCourse(generator))
	{
		if (randGenerator(generator) <= 0.49) //The change will be a right turn
		{
			cchange = 103.05195 + 46.42626*normGenerator(generator);
			return cchange;
		}
		else //The change will be a left turn
		{
			cchange = -98.53705 + 45.7462*normGenerator(generator);
			return cchange;
		}
	}
	else
		return 0.0;
}
Mover::~Mover(){}

