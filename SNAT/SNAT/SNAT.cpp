// SNATII.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"

//COLLISION AVOIDANCE LIBRARY


#include "Vessel.h"
#include "PathPlanner.h"
#include "PathMover.h"
#include "Trajectory.h"
#include "TrajectoryLoader.h"
#include "Constants.h"
#include "UnitTester.h"
#include "PathMoverTrajectoryBased.h"


//#include <dirent.h>
#include <cstdio> 
#include <random>
#include <Windows.h>
#include <sstream>
#include <ppl.h>


#define PI 3.14159

using namespace Concurrency;
using namespace std;
using namespace PathPlanner;
using namespace PathMover;
using namespace GeometryMethods;

using std::string;



void copyVessels(std::vector<Vessel> vesselVector, std::vector<Vessel> & copyVector)
{
	for (int copyIndex = 0; copyIndex < vesselVector.size(); copyIndex++)
	{
		string copyName = vesselVector[copyIndex].getName();
		double copyLength = vesselVector[copyIndex].getLength();
		double copyLongitude = vesselVector[copyIndex].getLongitude();
		double copyLatitude = vesselVector[copyIndex].getLatitude();
		double copyCourse = vesselVector[copyIndex].getCourse();
		double copySpeed = vesselVector[copyIndex].getSpeed();
		bool copyStatus = vesselVector[copyIndex].getStatus();
		Vessel copyVessel(copyName, copyLength, copyLongitude, copyLatitude, copyCourse, copySpeed, copyStatus);
		copyVector.push_back(copyVessel);
	}
}

void advanceOtherVessels(
	PathMoverTrajectoryBased &pattern_based_mover,
	vector<Vessel> &vesselVector, 
	double mt, 
	default_random_engine & generator,
	vector<TrajectoryPoint> &all_end_points,
	map<TrajectoryPoint, vector<Trajectory> > &endpoint_to_trajectory_map,
	map<string, VesselMinDistEntry> &id_pair_str_to_min_dist,
	ofstream & outSpeedF, 
	ofstream & outCourseF, 
	double loopTime,
	string avoidanceStr,
	ostream & outOwnHistoryF, 
	ostream & outTargetHistoryF) {

	//for (int i = 0; i < vesselVector.size(); i++)  {
	//	vesselVector[i].advanceRand(mt, generator, outSpeedF,
	//	outCourseF, loopTime, avoidanceStr, outOwnHistoryF, outTargetHistoryF);
	//}

	pattern_based_mover.advanceAllOtherVesselsPatternBased(
		mt,
		vesselVector,
		generator,
		all_end_points,
		endpoint_to_trajectory_map,
		id_pair_str_to_min_dist,
		outSpeedF,
		outCourseF,
		loopTime,
		avoidanceStr,
		outOwnHistoryF,
		outTargetHistoryF
		);
}


int main(int argc, char *argv[])
{
	time_t start_timer, end_timer;
	time(&start_timer);
	std::default_random_engine generator(SEEDNUM); //the seed helps to repeat the experiment

	std::string errorMsg;
	int numTest(0);
	std::cout << "Number of arguments received: " << argc << std::endl;
	try{
		if (argc < 1){
			errorMsg = "Invalid Arguments";
			throw errorMsg;
		}
		int numIteration(argc - 1);
		int countIteration(1);
		if (argc == 1)
			numIteration = argc;

		while (countIteration <= numIteration)
		{
			std::string fileTestName = "TestRun";
			if (argc == 1)
			{
				std::cout << "Input the desired Test Case: " << std::endl;
				std::cin >> numTest;
			}
			else{
				std::string numTestStr = argv[countIteration];
				numTest = atoi(numTestStr.c_str());
			}
			std::cout << "Test Number selected: " << numTest << std::endl;

			if (numTest <= 0 || numTest > 16){
				errorMsg = "Invalid Test Number";
				throw errorMsg;
			}
			std::stringstream ss;
			ss << numTest;
			std::string strAdd = ss.str();
			fileTestName += strAdd;
			fileTestName += ".txt";

			cout << "The fileTestName:" << fileTestName.c_str() << endl;

			//std::ifstream readFile(fileTestName, ios::binary | ios::ate);
			std::ifstream readFile(fileTestName);
			cout << "is open?" << readFile.is_open() << endl;
			cout << "end of file?" << readFile.eof() << endl;

			//cout << "size of the test file:" << fileTestName << " = " << readFile.tellg() << endl;

			std::string outFileNames;
			outFileNames = "OutConflicts";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outConflictF(outFileNames.c_str());
			outFileNames = "Collision";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outCollision(outFileNames.c_str());
			outFileNames = "CPA";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outCPA(outFileNames.c_str());
			outFileNames = "Manoeuvre";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outManeuvre(outFileNames.c_str());
			outFileNames = "Infor";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outInfor(outFileNames.c_str());
			outFileNames = "Check";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outCheck(outFileNames.c_str());
			outFileNames = "Manoeuvre";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outManoeuvreF(outFileNames.c_str());
			outFileNames = "Randspeed";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outSpeedF(outFileNames.c_str());
			outFileNames = "RandCourse";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outCourseF(outFileNames.c_str());
			outFileNames = "OwnHistory";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outOwnHistoryF(outFileNames.c_str());
			outFileNames = "TargetHistory";
			outFileNames += strAdd;
			outFileNames += ".txt";
			std::ofstream outTargetHistoryF(outFileNames.c_str());

			Mover * realEnv = new Mover();
			pathGenerator* CollisionAvd = new pathGenerator();

			string read; //Temp string to read line from txt
			vector<Vessel> vesselVector;
			vector<Collision> collisionVector;
			vector<Conflict> conflictVector;
			double safeDis = 0;
			int runTime = 24 * 60; // 24 hours
			double MT = 15; // minitor time in minutes
			double conditionT;
			double mt = 1; //increment time in minutes
			int numIndex = 0;
			double safeFactor = 2.0;
			double checkSafeDis;
			vector<double> numVector;

			while (!readFile.eof()) //Read file
			{
				getline(readFile, read);
				if (read != "") {
					string aName;
					double aLength, aLongitude, aLatitude, aCourse, aSpeed;
					istringstream is(read);
					is >> aName;
					is >> aLength;
					is >> aLongitude;
					is >> aLatitude;
					is >> aCourse;
					is >> aSpeed;
					Vessel aVessel(aName, aLength, aLongitude, aLatitude, aCourse, aSpeed, false);
					vesselVector.push_back(aVessel);
					//cout << "new vessel created from: " << read << endl;
					aVessel.printVessel();
				}
			}

			if (vesselVector.size() == 0) {
				cout << "no vessel data read in, skip this run" << endl;
				break;
			}

			/*Start the ABM for other vessels*/
			PathMoverTrajectoryBased pattern_based_mover = PathMoverTrajectoryBased(); 
			pattern_based_mover.clearAppendBasedFileStream(); // clear historical append based out file stream
			pattern_based_mover.prepareOutFileStreams(strAdd); // ofstream created
			pattern_based_mover.loadPatternRelatedData(); // load pattern data
			pattern_based_mover.prepareRandomEngine(); // prepare rand engine
			//pattern_based_mover.assignInitialPatternsToVessels(vesselVector); // assign initial pattern
			pattern_based_mover.assignPatternsToVesselsNearest(vesselVector); // assign pattern according to read in vessel lat, lon and course

			for (int i = 0; i < vesselVector.size(); i++) {
				vesselVector[i].printPatternRelated();
			}

			/*initial arrangement of vessels*/
			//pattern_based_mover.initialAlignmentForAllVesselsWrapper(vesselVector); // called if use assignInitialPatternsToVessel

			/*starting manoeuvering*/
			for (double loopTime = 0; loopTime <= runTime; loopTime += mt) //Simulate for 2 hours
			{
				time(&start_timer);
				pattern_based_mover.advanceAllOtherVesselsPatternBasedWrapper(mt, vesselVector, loopTime);
				time(&end_timer);
				cout << "simulation tick: " << loopTime << "/" << runTime
					<< ", time spent:" << difftime(end_timer, start_timer) << endl;
			}

			/*close stream to actually write to file */
			pattern_based_mover.closeOutFileStreams();

			/*stop for ABM only*/
			system("pause");
			return 0;

	//// testing return vector reference or value
	//TrajectoryPoint temp1(1.2, 103.2);
	//TrajectoryPoint temp2(1.2, 103.3);
	//TrajectoryPoint temp3(1.2, 103.4);
	//vector<TrajectoryPoint> temp_trajectory;
	//temp_trajectory.push_back(temp1);
	//temp_trajectory.push_back(temp2);
	//temp_trajectory.push_back(temp3);

	//Trajectory tr;
	//cout << "before adding one point into trajectory:" << tr.getTrajectoryLength() << endl;
	//tr.getPoints()->push_back(temp1);
	//cout << "after adding one point into trajectory:" << tr.getTrajectoryLength() << endl;
	//
	//// testing pushing back to vector a copied object or object reference? ans: a copied object
	//cout << "try to modify the trajectory point in vector" << endl;
	//temp1.setLatitude(1.3);
	//cout << "temp1 point" << temp1.getLatitude() << " , " << temp2.getLongitude() << endl;
	//cout << "in the vector now" << endl;
	//for (int i = 0; i < temp_trajectory.size(); i++) {
	//	cout << "temp_trajectory[" << i << "] = " << temp_trajectory[i].getLatitude() << " , " << temp_trajectory[i].getLongitude() << endl;
	//}

	//Trajectory tr_with_vector(temp_trajectory, 6); // temp_trajectory vector is passed by value in
	//cout << "pass by value trajecotory length:" << tr_with_vector.getTrajectoryLength() << endl;
	//tr_with_vector.getPoints()->push_back(TrajectoryPoint(1.2, 103.5)); // return by default is using by value, here use pointer to make changes
	//cout << "pass by value trajecotory length after push one more:" << tr_with_vector.getTrajectoryLength() << endl;
	//cout << "pass by value trajecotory length after push one more original vector:" << temp_trajectory.size() << endl;

	// testing read from sub folder
	//ifstream test_if("trajectory_data/temp.csv");
	//string nextline;
	//int count = 0;
	//while ( ! test_if.eof()) {
	//	getline(test_if, nextline);
	//	count++;
	//	while (nextline != "") {
	//		//cout << "new line:" << nextline << endl;
	//		getline(test_if, nextline);
	//	}
	//	cout << "start of new chunk marked by new line return" << endl;
	//}
	//UnitTester my_tester;
	TrajectoryLoader my_loader;
	
	//my_tester.unitTestLoadMMSIIds(my_loader);
	//my_tester.unitTestLoadTrajectories(my_loader);
	//my_tester.unitTestLoadEndpoints(my_loader);
	//my_tester.unitTestLoadMINDistance(my_loader);

	// test arctan2
	//double direction =  90 - (atan2(10, 0) * 180 / PI);
	//if (direction < 0) {
	//	direction += 360;
	//}
	//cout << "direction" << direction << endl;

	/* TODO: Load in the protocol trajectories, endpoints, [cluster:(protocol trajectory, cluster size)],
	[endpoint: protocol trajectory] and [endpoint: augmented trajectory] mapping */


	vector<Trajectory> all_patterns = my_loader.loadTrajectoryList(TRAJECTORY_INPUT_FOLDER_NAME + "/" +
		PROTOCOL_TRAJECTORIES_WITH_CLUSTER_SIZE_NAME);

	//for (int i = 0; i < all_patterns.size(); i++) {
	//	cout << "pattern " << i << "th id is:" << all_patterns[i].getId() << endl;
	//}

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



			/*Assume first of vesselVector is our own vessel, disabled for ABM*/
			//vesselVector[0].isOwnVessel = true; // vessel 0 will not be logged to OutTargetF
			bool randReaction = true;
			bool manoeuvre = true;

			safeDis = vesselVector[0].getLength() * 2;
			checkSafeDis = vesselVector[0].getLength()*safeFactor;

			/*initial arrangement of vessels*/
			pattern_based_mover.initialAlignmentForAllVessels(
				vesselVector,
				generator,
				endpoints,
				endpoints_to_protocol_mapping,
				id_pair_str_to_min_dist
				);

			/*starting manoeuvering*/
			if (manoeuvre == true)
			{
				for (double loopTime = 0; loopTime <= runTime; loopTime += mt) //Simulate for 2 hours
				{
					time(&start_timer);
	
					advanceOtherVessels(
						pattern_based_mover,
						vesselVector, 
						mt, 
						generator,
						endpoints,
						endpoints_to_protocol_mapping, 
						id_pair_str_to_min_dist, 
						outSpeedF, 
						outCourseF, 
						loopTime, 
						"NULL", 
						outOwnHistoryF, 
						outTargetHistoryF);

					time(&end_timer);
					cout << "simulation tick: " << loopTime << "/" << runTime
						<< ", time spent:" << difftime(end_timer, start_timer) << endl;
				}
			}

			/*stop for ABM only*/
			system("pause");
			return 0;

			if (manoeuvre == true)
			{
				for (double loopTime = 0; loopTime <= runTime; loopTime += mt) //Simulate for 2 hours
				{
					std::cout << "Current Real Time: " << loopTime << endl;
					vector<Vessel> copyVector;
					cout << "At time t = " << loopTime << "m" << endl;
					double ETC; //Earliest time to collision
					copyVessels(vesselVector, copyVector);
					ETC = CollisionAvd->findTime(vesselVector, copyVector, collisionVector, conflictVector,
						safeDis, MT, mt, loopTime, outConflictF,outCPA, outCollision);
  					string option = "Y";

					//eliminate those vessels that have collisions at the start
					if (loopTime == 0)
					{
						for (int i = 0; i < collisionVector.size(); i++)
						{
							for (int j = 0; j < vesselVector.size(); j++)
							{
								if (strcmp(vesselVector[j].getName().c_str(), collisionVector[i].getName().c_str()) == 0)
								{
									//eliminate vesselVector[j]
									vesselVector.erase(vesselVector.begin() + j);
									j--;
								}
							}
						}
						collisionVector.clear();
					}

					if (randReaction)
					{
						if (ETC <= 10)
							option = "Y";
						else
						{
							if (ETC <= 20)
							{
								if (randGenerator(generator) <= 0.7)
									option = "Y";
								else
									option = "N";
							}
							else
							{
								if (randGenerator(generator) <= 0.5)
									option = "Y";
								else
									option = "N";
							}
						}

					}
					conditionT = 15;

					if (ETC == 0)
					{
						cout << "Case 1: Collision detected cannot be avoid." << endl;
						// advance all other vessels, TODO: check in this case, if my own vessel is included?? since vesselVector contains it.
						advanceOtherVessels(
							pattern_based_mover,
							vesselVector, 
							mt, 
							generator, 
							endpoints, 
							endpoints_to_protocol_mapping,
							id_pair_str_to_min_dist, 
							outSpeedF, 
							outCourseF, 
							loopTime, 
							"NULL", 
							outOwnHistoryF, 
							outTargetHistoryF);
					}
					else
					{
						if (ETC != -1)
						{
							cout << "Case 2: Collision detected, search for solution." << endl;
							//cout << "Do you want to change your course now? (Y or N)" << endl;       
							//cin >> option;       
							if (option == "Y")
							{
								if (ETC > conditionT)
									conditionT = ETC;

								cout << "Case 3: Collision detected, solution accepted." << endl;
								std::string maneuvrePerf;
								//we run the algorithm to solve the conflicts characterized by ETC!
								std::vector<Conflict> vectorConf2Solve;
								int ii(0);
								double maxTime(ETC);
								for (int i = 0; i < conflictVector.size(); i++)
								{
									//if (conflictVector[i].getTCPA() <= ETC || (conflictVector[i].getTCPA() < 10))
									if (conflictVector[i].getTCPA() <= ETC)
									{
										vectorConf2Solve.push_back(conflictVector[i]);
										/*if (vectorConf2Solve[ii].getTCPA() >= maxTime)
										{
											maxTime = vectorConf2Solve[ii].getTCPA();
											ii++;
										}*/
									}
								}
								
								/*CollisionAvd->SolveCollision(vesselVector, collisionVector, vectorConf2Solve,
									checkSafeDis, conditionT,mt, loopTime, generator,
									outSpeedF, outCourseF,outManoeuvreF, maneuvrePerf, safeFactor);*/
								CollisionAvd->SolveCollision(vesselVector, collisionVector, vectorConf2Solve,
									checkSafeDis, maxTime, mt, loopTime, generator,
									outSpeedF, outCourseF, outManoeuvreF, maneuvrePerf, safeFactor);
								
								// advance all other vessels
								
								advanceOtherVessels(
									pattern_based_mover,
									vesselVector, 
									mt, 
									generator, 
									endpoints, 
									endpoints_to_protocol_mapping,
									id_pair_str_to_min_dist,
									outSpeedF, 
									outCourseF, 
									loopTime, 
									maneuvrePerf, 
									outOwnHistoryF, 
									outTargetHistoryF);
								
								vectorConf2Solve.clear();
								conflictVector.clear();
							}
							else
							{
								cout << "Case 3: Collision detected, solution ignored." << endl;
								// advance all other vessels
								advanceOtherVessels(
									pattern_based_mover,
									vesselVector, 
									mt, 
									generator,
									endpoints, 
									endpoints_to_protocol_mapping,
									id_pair_str_to_min_dist, 
									outSpeedF, 
									outCourseF, 
									loopTime, 
									"NULL", 
									outOwnHistoryF, 
									outTargetHistoryF);
							}
						}
						else
						{
							cout << "Case 5: No collision detected." << endl;
							// advance all other vessels
							advanceOtherVessels(
								pattern_based_mover,
								vesselVector, 
								mt, 
								generator,
								endpoints, 
								endpoints_to_protocol_mapping,
								id_pair_str_to_min_dist, 
								outSpeedF, 
								outCourseF, 
								loopTime, 
								"NULL", 
								outOwnHistoryF, 
								outTargetHistoryF);
						}
					}

					for (int index = 0; index < vesselVector.size(); index++)
						vesselVector[index].setStatus(false);
					copyVector.clear();
				}
			}
			else
			{
				vector<Vessel> copyVector;
				for (int copyIndex = 0; copyIndex < vesselVector.size(); copyIndex++)
				{
					string copyName = vesselVector[copyIndex].getName();
					double copyLength = vesselVector[copyIndex].getLength();
					double copyLongitude = vesselVector[copyIndex].getLongitude();
					double copyLatitude = vesselVector[copyIndex].getLatitude();
					double copyCourse = vesselVector[copyIndex].getCourse();
					double copySpeed = vesselVector[copyIndex].getSpeed();
					bool copyStatus = vesselVector[copyIndex].getStatus();
					Vessel copyVessel(copyName, copyLength, copyLongitude, copyLatitude, copyCourse, copySpeed, copyStatus);
					copyVector.push_back(copyVessel);
				}
				CollisionAvd->checkConflict(vesselVector, safeDis, runTime, mt, outCollision);
				copyVector.clear();
			}
			countIteration++;
			readFile.close();
			outConflictF.close();
			outCollision.close();
			outCPA.close();
			outManeuvre.close();
			outInfor.close();
			outCheck.close();
			outManoeuvreF.close();
			outSpeedF.close();
			outCourseF.close();
			outOwnHistoryF.close();
			outTargetHistoryF.close();
			vesselVector.clear();
			collisionVector.clear();
			numVector.clear();
		}
	}
	catch (std::string error)
	{
		std::cout << "Exception occurred: " << error << std::endl;
	}
	std::system("pause");
	return 0;

}

