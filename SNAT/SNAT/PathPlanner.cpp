#include "PathPlanner.h"
#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <iterator>

using namespace std;
using std::string;

//ofstream collisionFile ("Collision.txt", ios::app);
using namespace PathPlanner;
using namespace GeometryMethods;

Collision::Collision(string aName, double aCPA, double aTime, double aPercentage)
{
	Vessel_Name = aName;
	CPA = aCPA;
	Time = aTime;
	Percentage = aPercentage;
}
Collision::Collision()
{}

string Collision::getName(){ return Vessel_Name; }
double Collision::getCPA(){ return CPA; }
double Collision::getTime(){ return Time; }
double Collision::getPercentage(){ return Percentage; }
void Collision::setCPA(double newCPA){ CPA = newCPA; }
void Collision::setTime(double newTime){ Time = newTime; }
void Collision::setPercentage(double newPercentage){ Percentage = newPercentage; }
void Collision::printCollision()
{
	cout << "Vessel " << Vessel_Name << " violates own vessel's ship domain with closet distance of " << CPA << endl;
}
void Collision::outCollision(std::ofstream & collisionFile)
{
	collisionFile << Time << " " << Vessel_Name << " " << CPA << " " << Percentage << " " << endl;
}

Conflict::Conflict(string aName, double aCPA, double aTCPA, double aSafeDis)
{
	Vessel_Name = aName;
	CPA = aCPA;
	TCPA = aTCPA;
	SafeDis = aSafeDis;
	Percentage = (SafeDis - CPA) / SafeDis*1.00;
}

string Conflict::getName(){ return Vessel_Name; }
double Conflict::getCPA(){ return CPA; }
double Conflict::getTCPA() { return TCPA; }
double Conflict::getSafeDis(){ return SafeDis; }
double Conflict::getPercentage(){ return Percentage; }
void Conflict::outConflict(int currentTime, std::ofstream & conflictFile)
{
	//cout << "Vessel " << Vessel_Name << " violates own vessel's ship domain with closet distance of " << CPA << "m " << "in "<< TCPA << " min." << endl;
	//cout << "Degree of violation is " << Percentage  << "." << endl; 
	conflictFile << currentTime << " " << Vessel_Name << " " << CPA << "m " << TCPA << "min " << Percentage << "." << endl;
}

pathGenerator::pathGenerator(void)
{};
double pathGenerator::getDistance(double LongitudeA, double LatitudeA, double LongitudeB, double LatitudeB) //To calculate distance between two coordinates
{
	//positive coordinates indicate N/W for longitude/latitude while negative coordinates indicate S/E
	double diffLo = toRadius(LongitudeA) - toRadius(LongitudeB);
	double diffLa = toRadius(LatitudeA) - toRadius(LatitudeB);
	double constantA = pow(sin(diffLo / 2), 2) +
		cos(toRadius(LatitudeA))*cos(toRadius(LatitudeB))*pow(sin(diffLo / 2), 2);
	double constantB = 2 * asin(sqrt(constantA));
	double distance = 6371000 * constantB;
	return distance;
}

double pathGenerator::findTime(vector<Vessel>& A_vesselVector, vector<Vessel> A_copyVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector,
	double safeDis, double MT, double mt, int currentTime,
	std::ofstream & outConflictF, std::ofstream & outCPAF, std::ofstream & outCollisionF)
{
	double TCPA = 0;

	for (double time = 0; time <= MT; time += mt) //time in mins
	{
		if (time != 0)
		{
			for (int index = 0; index < A_copyVector.size(); index++) //Model vessel movement in time intervals
			{
				double x_coordinate = A_copyVector[index].getSpeed()*1.852 / 60 * sin(toRadius(A_copyVector[index].getCourse()))*mt;
				double y_coordinate = A_copyVector[index].getSpeed()*1.852 / 60 * cos(toRadius(A_copyVector[index].getCourse()))*mt;
				double newLongitude = A_copyVector[index].getLongitude() + 180 / PI / cos(toRadius(A_copyVector[index].getLatitude()))*x_coordinate / 6371;
				double newLatitude = A_copyVector[index].getLatitude() + 180 / PI*y_coordinate / 6371;
				A_copyVector[index].setLongitude(newLongitude);
				A_copyVector[index].setLatitude(newLatitude);
			}
		}
		//Concurrency::parallel_for (int(1),int (A_vesselVector.size()),[&] (int j)
		//{
		for (int j = 1; j < A_vesselVector.size(); j++)
		{
			//std::cout << "vessel: " << j << endl;
			if (A_vesselVector[j].getStatus() == false)
			{
				double aDistance = getDistance(A_copyVector[0].getLongitude(), A_copyVector[0].getLatitude(), 
					A_copyVector[j].getLongitude(), A_copyVector[j].getLatitude());
				if (aDistance <= safeDis)
				{
					A_vesselVector[j].setStatus(true);
					if (time >= 6)//create a conflict
					{
						Conflict aConflict(A_copyVector[j].getName(), aDistance, time, safeDis);
						conflictVector.push_back(aConflict);
						outConflictF << currentTime << " " << aConflict.getName() << " " << aConflict.getCPA() << " m " << aConflict.getTCPA() << " m " << aConflict.getPercentage() << "." << endl;
						aConflict.outConflict(currentTime, outCPAF);
					}
					//if the time is 0 then create a collision instead
					else
					{
						bool update = false;
						std::vector<bool> verified;
						Collision aCollision(A_vesselVector[j].getName(), aDistance, currentTime, (safeDis - aDistance) / safeDis*1.0);
						for (int checkIndex = 0; update == false && checkIndex < A_collisionVector.size(); checkIndex++)
						{
							if (aCollision.getName() == A_collisionVector[checkIndex].getName())
							{
								A_collisionVector[checkIndex].setCPA(aCollision.getCPA());
								A_collisionVector[checkIndex].setTime(aCollision.getTime());
								A_collisionVector[checkIndex].setPercentage(aCollision.getPercentage());
								update = true;
							}
						}
						if (update == false)
						{
							A_collisionVector.push_back(aCollision);
							aCollision.outCollision(outCollisionF);	
						}
					}
				}
			}
		}//);           
	}

	/*int findIndex;
	int searchIndex;
	if (!conflictVector.empty())
	{
		for (searchIndex = 0; searchIndex < conflictVector.size() && conflictVector[searchIndex].getTCPA() <= 6; searchIndex++)
		{

			if (conflictVector[searchIndex].getTCPA() != 0)
			{
				bool update = false;
				Collision aCollision(conflictVector[searchIndex].getName(), conflictVector[searchIndex].getCPA(), conflictVector[searchIndex].getTCPA() + currentTime, (safeDis - conflictVector[searchIndex].getCPA()) / safeDis*1.0);
				for (int checkIndex = 0; update == false && checkIndex < A_collisionVector.size(); checkIndex++)
				{
					if (aCollision.getName() == A_collisionVector[checkIndex].getName())
					{
						A_collisionVector[checkIndex].setCPA(aCollision.getCPA());
						A_collisionVector[checkIndex].setTime(aCollision.getTime());
						A_collisionVector[checkIndex].setPercentage(aCollision.getPercentage());
						update = true;
					}
				}
				if (update == false)
				{
					A_collisionVector.push_back(aCollision);
					aCollision.outCollision(outCollisionF);
				}
			}

		}

		findIndex = searchIndex;*/
	for (int checkIndex = 0; checkIndex < A_collisionVector.size(); checkIndex++)
	{
		//In case a collision time has not been updated it means that the collision does not exist anymore,
		//therefore, we eliminate it
		if (A_collisionVector[checkIndex].getTime() < currentTime)
		{
			A_collisionVector.erase(A_collisionVector.begin() + checkIndex);
		}
	}
	/*if (findIndex < conflictVector.size() && conflictVector[findIndex].getTCPA() != 0)*/
	if (conflictVector.size()>0)
	{
		TCPA = conflictVector[0].getTCPA();
		cout << "Warning! The distance between " << A_vesselVector[0].getName() << " and "
			<< conflictVector[0].getName() << " after " << TCPA << " minutes is: "
			<< conflictVector[0].getCPA() << "m" << endl;

		return TCPA;
	}
	else if (A_collisionVector.size()>0 && conflictVector.size()==0)
	{
		cout << "Only Collision(s) recorded, which cannot be avoided!" << endl;
		return 0;
	}
	//}
	else if (conflictVector.empty())
		return -1;
}

bool pathGenerator::checkFuture(vector<Vessel> A_vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector, double safeDis, double minDist,
	double conditionT, double mt, int currentTime, std::default_random_engine & gen,
	std::ofstream & outSpeedF, std::ofstream & outCourseF)
{
	int numReplications = 10;
	std::vector<int> collision;// = 0;
	std::vector<bool> check;
	std::vector<std::map<std::pair<int,std::string>, std::pair<bool,double> > > check_collisions;
	
	check.resize(numReplications);

	collision.resize(numReplications);
	check_collisions.resize(numReplications);
	for (int replicate = 0; replicate < numReplications; replicate++)
	{
		check[replicate] = true;
	}
	Concurrency::parallel_for(int(0), int(numReplications), [&](int replicate)
	//for (int replicate=0;replicate<numReplications;replicate++)
	{
		vector<Vessel> copyVector;
		for (int i = 0; i < A_vesselVector.size(); i++)
		{
			Vessel aVessel = A_vesselVector[i];
			copyVector.push_back(aVessel);
		}
		//for (double time = 0; check == true && time <= conditionT; time += mt) //time in mins
		//Start from the time =1 so you advance according to your new condition
		double time = 0.0;
		while (time <= conditionT && check[replicate] == true)
		{
			if (time != 0)
			{
				for (int v = 0; v < copyVector.size(); v++)
				{
					copyVector[v].advanceRand(mt, gen, outSpeedF, outCourseF,time, "NO");
				}
			}

			for (int j = 1; check[replicate] == true && j < copyVector.size(); j++)
			{
				double aDistance = getDistance(copyVector[0].getLongitude(), copyVector[0].getLatitude(), copyVector[j].getLongitude(), copyVector[j].getLatitude());
				if (time == 1 && j == 1)
					minDist = aDistance;
				else if (aDistance <= minDist)
					minDist = aDistance;
				bool recorded = false;
				for (int search = 0; recorded == false && search < A_collisionVector.size(); search++)
				{
					//we check this is not a one we can actually avoid!!!
					if (copyVector[j].getName() == A_collisionVector[search].getName())
						recorded = true;
				}
				//if (recorded == false && aDistance <= safeDis
				if (recorded == false && aDistance <= safeDis)
				{
					check[replicate] = false;
				}
				//else if (recorded == true)
				//{
				//	double OwnCourse(copyVector[0].getCourse());
				//	double TargetCourse(copyVector[j].getCourse());
				//	//compute the angle between the conflicting vessels:
				//	if (copyVector[0].getCourse() > 180 &&  copyVector[0].getCourse()< 360.0)
				//		OwnCourse -= 180.00;
				//	else
				//		OwnCourse = copyVector[0].getCourse();
				//	if (copyVector[j].getCourse() > 180 && copyVector[j].getCourse()< 360.0)
				//		TargetCourse -= 180.00;
				//	else
				//		TargetCourse = copyVector[j].getCourse();
				//	double angleBetween(0.0);
				//	if (TargetCourse > OwnCourse)
				//	{
				//		angleBetween = TargetCourse - OwnCourse;
				//	}
				//	else
				//		angleBetween = -TargetCourse + OwnCourse;
				//	double collisionDist(sqrt(pow((copyVector[0].getLength()/2*abs(sin(copyVector[0].getCourse())) - 
				//		copyVector[j].getLength()/2*abs(sin(copyVector[j].getCourse()))),2)+
				//		pow((copyVector[0].getLength() / 2 * abs(cos(copyVector[0].getCourse())) +
				//		copyVector[j].getLength() / 2 * abs(cos(copyVector[j].getCourse()))), 2)));
				//	//need to avoid  contact
				//	
				//	if (aDistance <= collisionDist)
				//	{
				//		check[replicate] = false;
				//		std::pair<std::pair<int, std::string>, std::pair<bool,double> > mypair;
				//		mypair.first.first = j;
				//		mypair.first.second = copyVector[j].getName();
				//		mypair.second.first = false;
				//		mypair.second.second = time;
				//		check_collisions[replicate].insert(mypair);

				//	}
				//}
			}
			time = time + mt;
		}

		if (check[replicate] == false)
			collision[replicate] = 1;
	}
	);
	int sum_of_elt = std::accumulate(collision.begin(), collision.end(), 0);
	if (sum_of_elt <= 1)
	return true;
	else
		return false;
	check.clear();
	collision.clear();
	check_collisions.clear();
}

void pathGenerator::advance(vector<Vessel>& A_vesselVector, double mt)
{

	for (int index = 0; index < A_vesselVector.size(); index++) //Model vessel movement in time intervals of 5 minutes
	{

		double x_coordinate = A_vesselVector[index].getSpeed()*1.852 / 60 * sin(toRadius(A_vesselVector[index].getCourse()))*mt;
		double y_coordinate = A_vesselVector[index].getSpeed()*1.852 / 60 * cos(toRadius(A_vesselVector[index].getCourse()))*mt;
		double newLongitude = A_vesselVector[index].getLongitude() + 180 / PI / cos(toRadius(A_vesselVector[index].getLatitude()))*x_coordinate / 6371;
		double newLatitude = A_vesselVector[index].getLatitude() + 180 / PI*y_coordinate / 6371;
		A_vesselVector[index].setLongitude(newLongitude);
		A_vesselVector[index].setLatitude(newLatitude);

	}
	return;
}

void pathGenerator::checkConflict(vector<Vessel>& A_vesselVector, double safeDis, int runTime, double mt,
	std::ofstream & outCollisionF)
{
	vector<Collision> collisionVector;

	int countCollision;

	for (double time = 0; time <= runTime; time += mt)
	{
		vector<Collision> tempcollisionVector;
		tempcollisionVector.resize(A_vesselVector.size());
		countCollision = 0;
		Concurrency::parallel_for(int(1), int(A_vesselVector.size()), [&](int j)
			//for (int j = 1; j < A_vesselVector.size(); j++)
		{

			double aDistance = getDistance(A_vesselVector[0].getLongitude(), A_vesselVector[0].getLatitude(), A_vesselVector[j].getLongitude(), A_vesselVector[j].getLatitude());

			if (aDistance <= safeDis && A_vesselVector[j].getStatus() == false)
			{
				Collision aCollision(A_vesselVector[j].getName(), aDistance, time, (safeDis - aDistance) / safeDis);
				//collisionVector.push_back(aCollision);
				tempcollisionVector[j] = aCollision;
				aCollision.outCollision(outCollisionF);
				A_vesselVector[j].setStatus(true);
				countCollision++;
			}
		});
		advance(A_vesselVector, mt);
		if (countCollision>0)
		{
			for (int jj = 0; jj<countCollision; jj++)
				collisionVector.push_back(tempcollisionVector[jj]);
		}
		tempcollisionVector.clear();
	}
}

bool pathGenerator::changeSpeed(vector<Vessel>& A_vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector, double aSafeDis,
	double conditionT, double mt, int currentTime, std::default_random_engine & gen,
	std::ofstream & outSpeedF, std::ofstream & outCourseF, std::ofstream & outManeuvreF, Manoeuvre & aManoeuvre, std::string & outmsg)
{
	double storeCourse = A_vesselVector[0].getCourse();
	double storeSpeed = A_vesselVector[0].getSpeed();
	std::cout << "Tryin' with speed an course: " << std::endl;
	double minDistStart(0.0);
	double minDist(0.0);
	double upcc(0.0);
	double deltacc(0.0);
	/*if (A_collisionVector.size()>0)
	{
		upcc = 60;
		deltacc = 1.0;
	}*/
	/*else
	{*/
		upcc = 60;
		deltacc = 1.0;
	/*}*/
	for (int schange = -2; schange >= -10 && A_vesselVector[0].getSpeed() > 1; schange--)
	{
		for (double cchange = 15; cchange <= upcc; cchange += deltacc)
		{
			A_vesselVector[0].setSpeed(storeSpeed + schange);
			A_vesselVector[0].setCourse(storeCourse + cchange);
			std::cout << "Change Course, Change Speed: " << cchange << "," << schange << std::endl;
			if (checkFuture(A_vesselVector, A_collisionVector, conflictVector, aSafeDis, minDist, conditionT, mt,
				currentTime, gen, outSpeedF, outCourseF))
			{
				cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
				cout << "Followed by a speed change of " << schange << " knots" << endl;
				outmsg = "Delta_C[degrees]: ";
				outmsg += to_string(cchange);
				outmsg += " Delta_S[Knots]: ";
				outmsg += to_string(schange);
				aManoeuvre = Manoeuvre(currentTime, cchange, schange);
				aManoeuvre.outManoeuvre(outManeuvreF);
				return true;
			}
			else if (minDist > minDistStart)
			{
				int currentcchange(cchange);
				int currentschange(schange);
				minDistStart = minDist;
			}
		}
		for (double cchange = -15; cchange >= -upcc; cchange -= deltacc)
		{
			A_vesselVector[0].setSpeed(storeSpeed + schange);
			A_vesselVector[0].setCourse(storeCourse + cchange);
			std::cout << "Change Course, Change Speed: " << cchange << "," << schange << std::endl;
			if (checkFuture(A_vesselVector, A_collisionVector, conflictVector, aSafeDis, minDist, conditionT, mt,
				currentTime, gen, outSpeedF, outCourseF))
			{
				cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
				cout << "Followed by a speed change of " << schange << " knots" << endl;
				outmsg = "Delta_C[degrees]: ";
				outmsg += to_string(cchange);
				outmsg += " Delta_S[Knots]: ";
				outmsg += to_string(schange);
				aManoeuvre = Manoeuvre(currentTime, cchange, schange);
				aManoeuvre.outManoeuvre(outManeuvreF);
				return true;
			}
			else if (minDist > minDistStart)
			{
				int currentcchange(cchange);
				int currentschange(schange);
				minDistStart = minDist;
			}
		}
	}

	for (int schange = 2; schange <= 10; schange++)
	{
		for (double cchange = 15; cchange <= upcc; cchange += deltacc)
		{
			A_vesselVector[0].setSpeed(storeSpeed + schange);
			A_vesselVector[0].setCourse(storeCourse + cchange);
			std::cout << "Change Course, Change Speed: " << cchange << "," << schange << std::endl;
			if (checkFuture(A_vesselVector, A_collisionVector, conflictVector, aSafeDis, minDist, conditionT, mt,
				currentTime, gen, outSpeedF, outCourseF))
			{
				cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
				cout << "Followed by a speed change of " << schange << " knots" << endl;
				outmsg = "Delta_C[degrees]: ";
				outmsg += to_string(cchange);
				outmsg += " Delta_S[Knots]: ";
				outmsg += to_string(schange);
				aManoeuvre = Manoeuvre(currentTime, cchange, schange);
				aManoeuvre.outManoeuvre(outManeuvreF);
				return true;
			}
			else if (minDist > minDistStart)
			{
				int currentcchange(cchange);
				int currentschange(schange);
				minDistStart = minDist;
			}
		}
		for (double cchange = -15; cchange >= -upcc; cchange -= deltacc)
		{
			A_vesselVector[0].setSpeed(storeSpeed + schange);
			A_vesselVector[0].setCourse(storeCourse + cchange);
			std::cout << "Change Course, Change Speed: " << cchange << "," << schange << std::endl;
			if (checkFuture(A_vesselVector, A_collisionVector, conflictVector, aSafeDis,minDist,
				conditionT, mt,	currentTime, gen, outSpeedF, outCourseF))
			{
				cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
				cout << "Followed by a speed change of " << schange << " knots" << endl;
				outmsg = "Delta_C[degrees]: ";
				outmsg += to_string(cchange);
				outmsg += " Delta_S[Knots]: ";
				outmsg += to_string(schange);
				aManoeuvre = Manoeuvre(currentTime, cchange, schange);
				aManoeuvre.outManoeuvre(outManeuvreF);
				return true;
			}
		}
	}
	A_vesselVector[0].setCourse(storeCourse);
	A_vesselVector[0].setSpeed(storeSpeed);
	return false;
}
	bool pathGenerator::changeCourse(vector<Vessel>& A_vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector, double aSafeDis,
		double conditionT, double mt, int currentTime, std::default_random_engine & gen,
		std::ofstream & outSpeedF, std::ofstream & outCourseF, std::ofstream & outManeuvreF, Manoeuvre & aManoeuvre, std::string & outmsg)
{
	std::cout << "Tryin' course change: " << std::endl;
	double minDist(0.0);
	double storeCourse = A_vesselVector[0].getCourse();
	double upcc(0.0);
	double deltacc(0.0);
	/*if (A_collisionVector.size()>0)
	{
		upcc = 180;
		deltacc = 0.2;
	}*/
	/*else
	{*/
		upcc = 60;
		deltacc = 1.0;
	/*}*/
	for (double change = 15; change <= upcc; change += deltacc)
	{
		std::cout << "Change: " << change << std::endl;
		A_vesselVector[0].setCourse(storeCourse + change);
		if (checkFuture(A_vesselVector, A_collisionVector, conflictVector, aSafeDis,minDist, conditionT, mt,
			currentTime, gen, outSpeedF, outCourseF))
		{
			cout << "Delat_C [deg]: " << change << endl;
			outmsg = " Delat_C [deg]: ";
			outmsg += to_string(change);
			Manoeuvre aManoeuvre(currentTime, change, 0);
			aManoeuvre.outManoeuvre(outManeuvreF);
			return true;
		}
	}
	A_vesselVector[0].setCourse(storeCourse);
	for (double change = -15; change >= -upcc; change -= deltacc)
	{
		std::cout << "Change: " << change << std::endl;
		A_vesselVector[0].setCourse(storeCourse + change);
		if (checkFuture(A_vesselVector, A_collisionVector, conflictVector, aSafeDis,minDist, conditionT, mt,
			currentTime, gen, outSpeedF, outCourseF))
		{
			cout << "Delta_C[degrees]: " << change << endl;
			outmsg = " Delta_C[degrees]: ";
			outmsg += to_string(change);
			Manoeuvre aManoeuvre(currentTime, change, 0);
			aManoeuvre.outManoeuvre(outManeuvreF);
			return true;
		}
	}
	
	A_vesselVector[0].setCourse(storeCourse);
	if (changeSpeed(A_vesselVector, A_collisionVector, conflictVector, aSafeDis, conditionT, mt,
		currentTime, gen, outSpeedF, outCourseF, outManeuvreF, aManoeuvre, outmsg))
		return true;
	else
		return false;
}
	bool pathGenerator::SolveCollision(std::vector<Vessel> & vesselVector, vector<Collision>& A_collisionVector, vector<Conflict> & conflictVector,
		double checkSafeDis, double conditionT, double mt, double loopTime, std::default_random_engine & generator,
		std::ofstream & outSpeedF, std::ofstream & outCourseF, std::ofstream & outManoeuvreF, std::string & maneuvrePerf,
		double safeFactor)
	{
		bool feasible(false);
		Manoeuvre aManoeuvre;
		feasible = changeCourse(vesselVector, A_collisionVector, conflictVector, checkSafeDis, conditionT, mt,
			loopTime, generator, outSpeedF, outCourseF, outManoeuvreF, aManoeuvre, maneuvrePerf);
		while (!feasible && safeFactor > 1.5)
		{
			safeFactor -= 0.2;
			//std::cout << "Current Safety Factor: " << safeFactor << endl;
			checkSafeDis = vesselVector[0].getLength()*safeFactor;
			feasible = changeCourse(vesselVector, A_collisionVector, conflictVector, checkSafeDis, conditionT, mt, loopTime,
				generator, outSpeedF, outCourseF, outManoeuvreF, aManoeuvre, maneuvrePerf);
		}
		if (feasible == 0)
		{
			//implement safest maneuvre
			vesselVector[0].setCourse(vesselVector[0].getCourse() + aManoeuvre.getCourse());
			vesselVector[0].setSpeed(vesselVector[0].getSpeed() + aManoeuvre.getSpeed());
		}
		return feasible;
	}