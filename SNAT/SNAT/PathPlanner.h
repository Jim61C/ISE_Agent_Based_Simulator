
////STANDARD LIBRARY
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <time.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <algorithm>
#include <climits>
#include <numeric>

#include "PathMover.h"
#include "ppl.h""

using namespace std;
using namespace PathMover;
using namespace Concurrency;
using std::string;

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

namespace PathPlanner{
	class Collision {

	private:
		string Vessel_Name;
		double CPA;
		double Time;
		double Percentage;

	public:
		Collision();
		Collision(string, double, double, double); //Constructor
		string getName();
		double getCPA();
		double getTime();
		double getPercentage();
		void setCPA(double);
		void setTime(double);
		void setPercentage(double);
		void printCollision();
		void outCollision(std::ofstream & outFile);
	};

	class Conflict {

	private:
		string Vessel_Name;
		double CPA;
		double TCPA;
		double SafeDis;
		double Percentage;

	public:
		Conflict(string, double, double, double); //Constructor
		string getName();
		double getCPA();
		double getTCPA();
		double getSafeDis();
		double getPercentage();
		void outConflict(int time, std::ofstream & conflictFile);

	};

	//The path generator is class of methods.
	//It takes as input a trajectory and verifies whether there are conflicts
	class pathGenerator{
	public:
		pathGenerator();//default constructor
	private:
		double getDistance(double LongitudeA, double LatitudeA, double LongitudeB, double LatitudeB); //To calculate distance between two coordinates
		
		bool checkFuture(vector<Vessel> A_vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector, double safeDis, double minDist,
			double conditionT, double mt, int currentTime, std::default_random_engine & gen,
			std::ofstream & outSpeedF, std::ofstream & outCourseF);
		void advance(vector<Vessel>& A_vesselVector, double mt);
		bool changeSpeed(vector<Vessel>& A_vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector, double aSafeDis,
			double conditionT, double mt, int currentTime, std::default_random_engine & gen,
			std::ofstream & outSpeedF, std::ofstream & outCourseF, std::ofstream & outManeuvreF, Manoeuvre & aManoeuvre, std::string & outmsg);
		
		bool changeCourse(vector<Vessel>& A_vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector, double aSafeDis,
			double conditionT, double mt, int currentTime, std::default_random_engine & gen,
			std::ofstream & outSpeedF, std::ofstream & outCourseF, std::ofstream & outManeuvreF, Manoeuvre & aManoeuvre, std::string & outmsg);
		public:
			double findTime(vector<Vessel>& A_vesselVector, vector<Vessel> A_copyVector, vector<Collision>& A_collisionVector, 
				vector<Conflict> & conflictVector,
				double safeDis, double MT, double mt, int currentTime,
				std::ofstream & outConflictF, std::ofstream & outCPAF, std::ofstream & outCollisionF);
			void checkConflict(vector<Vessel>& A_vesselVector, double safeDis, int runTime, double mt,
				std::ofstream & outCollisionF);
			bool SolveCollision(std::vector<Vessel> & vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector,
				double checkSafeDis, double conditionT, double mt, double loopTime, std::default_random_engine & generator,
				std::ofstream & outSpeedF, std::ofstream & outCourseF, std::ofstream & outManoeuvreF, std::string & maneuvrePerf,
				double safeFactor);
	};
}
#endif