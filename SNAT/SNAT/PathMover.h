#include <iostream>
#include <string>
#include "geometry.h"
#include "Vessel.h"
using namespace std;
using std::string;

#ifndef PATHMOVER_H
#define PATHMOVER_H
#define PI 3.14159

namespace PathMover{
	class Manoeuvre {

	private:
		int Time;
		double Coursechange;
		double Speedchange;

	public:
		Manoeuvre();//default constructor
		Manoeuvre(int, double, double); //Constructor
		int getTime();
		double getCourse();
		double getSpeed();
		void printManoeuvre();
		void outManoeuvre(std::ofstream & outManeuvreF);

	};
	class Randcourse {

	private:
		string Vessel_Name;
		double Original;
		double Change;
		double New;

	public:
		Randcourse(string, double, double, double); //Constructor
		string getName();
		double getOriginal();
		double getChange();
		double getNew();
		void printRandcourse();
		void outRandcourse(std::ofstream & outRandCourse);

	};
	class Randspeed {

	private:
		string Vessel_Name;
		double Original;
		double Change;
		double New;

	public:
		Randspeed(string, double, double, double); //Constructor
		string getName();
		double getOriginal();
		double getChange();
		double getNew();
		void printRandspeed();
		void outRandspeed(std::ofstream & outRandSpeedF);

	};

	class Mover{
		
	public:
		Mover();
		double randSpeedChange(std::default_random_engine & generator);
		double randCourseChange(std::default_random_engine & generator);
		~Mover();

	};

}
#endif