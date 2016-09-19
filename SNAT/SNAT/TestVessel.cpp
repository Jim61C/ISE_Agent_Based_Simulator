#include "Vessel.h"
#include "Conflict.h"
#include "Collision.h"
#include "Manoeuvre.h"
#include "Randspeed.h"
#include "Randcourse.h"
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
//#include <dirent.h>
#include <cstdio> 
#include <random>
#include <Windows.h>
#include <ppl.h>


#define PI 3.14159

using namespace Concurrency;
using namespace std;
using std::string;



double toRadius (double degree) //Unit conversion from degree to radius
{
    return degree * PI / 180.0;       
}

double randGenerator (std::default_random_engine & gen)
{

	//std::default_random_engine generator(12345); //the seed helps to repeat the experiment
	std::uniform_real_distribution<double> distribution(0,1);
	//double aRand = distribution(generator);
	double aRand = distribution(gen);
	return aRand;
}

bool randSpeed (std::default_random_engine & gen)
{
    if (randGenerator (gen) <= 0.12) //There will be a random change in speed
    return true;
    else //There will not be a random change in speed
    return false;         
            
}

bool randCourse (std::default_random_engine & gen)
{    
   if (randGenerator (gen) <= 0.37) //There will be a random change in course
   return true;
   else //There will not be a random change in course
   return false;      
}

double normGenerator(std::default_random_engine & gen) //Use two random numbers to generate a random number that follows the standard normal distribution
{
    double normRand = sqrt(-2*2.303*log(randGenerator (gen)))*cos(2*PI*randGenerator (gen));
    return normRand;
}

double lognormGenerator(double mean, double std,std::default_random_engine & gen)
{
    double normRand = normGenerator(gen);
    double lognormRand = exp(mean + std*normRand);
    return lognormRand;   
}
       
double randSpeedChange(std::default_random_engine & generator)
{
    double schange; //Percentage of change in speed
    if (randSpeed(generator))
    {
      
      if (randGenerator (generator) <= 0.27) //The change will be an increase in speed
      {
       schange = lognormGenerator (-0.99487,0.77507,generator);
       return schange;         
      }
      else //The change will be a decrease in speed
      {
       schange = -0.57881+0.19179*normGenerator(generator);    
       return schange;          
      }              
    } 
    else 
    return 0.0;   
}

double randCourseChange(std::default_random_engine & generator)
{
    double cchange; //Change in course in degree
    if (randCourse(generator))
    {
      if (randGenerator (generator) <= 0.49) //The change will be a right turn
      {
       cchange = 103.05195+46.42626*normGenerator(generator);
       return cchange;         
      }
      else //The change will be a left turn
      {
       cchange = -98.53705+45.7462*normGenerator(generator);    
       return cchange;          
      }              
    } 
    else 
    return 0.0;   
}

void advanceRand (vector<Vessel>& A_vesselVector,double mt,std::default_random_engine & gen,std::ofstream & outSpeedF, std::ofstream & outCourseF)
{
     vector<Randspeed> speedVector;
     vector<Randcourse> courseVector;
     
	// Concurrency::parallel_for (int(0),int (A_vesselVector.size()),[&] (int index)
	 //{
		 for (int index = 1; index < A_vesselVector.size();index++) //Model random change in speed and course
		 {
		   double schange = randSpeedChange(gen);
		   double newSpeed = A_vesselVector[index].getSpeed()+schange*A_vesselVector[index].getSpeed();
		   Randspeed aRandspeed (A_vesselVector[index].getName(), A_vesselVector[index].getSpeed(),schange, newSpeed);
		   aRandspeed.outRandspeed(outSpeedF);
		   speedVector.push_back(aRandspeed);
		   A_vesselVector[index].setSpeed(newSpeed);
   
		   double cchange = randCourseChange(gen);
		   double newCourse = A_vesselVector[index].getCourse()+cchange;
		   if (newCourse > 360)
		   newCourse = newCourse - 360;
		   if (newCourse < 0)
		   newCourse = 360 + A_vesselVector[index].getCourse()+cchange;
		   Randcourse aRandcourse (A_vesselVector[index].getName(), A_vesselVector[index].getCourse(),cchange, newCourse);
		   aRandcourse.outRandcourse(outCourseF);
		   courseVector.push_back(aRandcourse);
		   A_vesselVector[index].setCourse(newCourse);
       
		 }
	 //});
     for (int index = 0; index < A_vesselVector.size(); index++) //Model vessel movement in time intervals of mt minutes
     { 
          
       double x_coordinate = A_vesselVector[index].getSpeed()*1.852/60*sin(toRadius(A_vesselVector[index].getCourse()))*mt;
       double y_coordinate = A_vesselVector[index].getSpeed()*1.852/60*cos(toRadius(A_vesselVector[index].getCourse()))*mt;
       double newLongitude = A_vesselVector[index].getLongitude() + 180/PI/cos(toRadius(A_vesselVector[index].getLatitude()))*x_coordinate/6371;
       double newLatitude = A_vesselVector[index].getLatitude() + 180/PI*y_coordinate/6371;
       A_vesselVector[index].setLongitude(newLongitude);
       A_vesselVector[index].setLatitude (newLatitude);
      
     }
     return;   
}


double getDistance (double LongitudeA, double LatitudeA, double LongitudeB, double LatitudeB) //To calculate distance between two coordinates
{
    //positive coordinates indicate N/W for longitude/latitude while negative coordinates indicate S/E
    double diffLo = toRadius(LongitudeA) - toRadius(LongitudeB);
    double diffLa = toRadius(LatitudeA) - toRadius(LatitudeB);
    double constantA = sin(diffLo/2)*sin(diffLo/2)+cos(toRadius(LongitudeA))*cos(toRadius(LongitudeB))*sin(diffLa/2)*sin(diffLa/2);
    double constantB = 2*asin(sqrt(constantA));     
    double distance = 6371000 * constantB;  
    return distance;
}

double findTime (vector<Vessel>& A_vesselVector, vector<Vessel> A_copyVector, vector<Collision>& A_collisionVector,
	double safeDis, double MT, double mt, int currentTime,
	std::ofstream & outConflictF, std::ofstream & outCPAF,std::ofstream & outCollisionF)
{
    
    vector<Conflict> conflictVector;
    double TCPA = 0;
    
    for (double time = 0; time <= MT; time += mt) //time in mins
    {
        for (int index = 0; index < A_copyVector.size(); index++) //Model vessel movement in time intervals
        { 
          if( time != 0)
          {  
           double x_coordinate = A_copyVector[index].getSpeed()*1.852/60*sin(toRadius(A_copyVector[index].getCourse()))*mt;
           double y_coordinate = A_copyVector[index].getSpeed()*1.852/60*cos(toRadius(A_copyVector[index].getCourse()))*mt;
           double newLongitude = A_copyVector[index].getLongitude() + 180/PI/cos(toRadius(A_copyVector[index].getLatitude()))*x_coordinate/6371;
           double newLatitude = A_copyVector[index].getLatitude() + 180/PI*y_coordinate/6371;
           A_copyVector[index].setLongitude(newLongitude);
           A_copyVector[index].setLatitude (newLatitude);
          }
        }
		//Concurrency::parallel_for (int(1),int (A_vesselVector.size()),[&] (int j)
		//{
        for (int j = 1; j < A_vesselVector.size(); j++)
        {
			//std::cout << "vessel: " << j << endl;
			if (A_vesselVector[j].getStatus() == false)
			{  
				double aDistance = getDistance (A_copyVector[0].getLongitude(), A_copyVector[0].getLatitude(), A_copyVector[j].getLongitude(), A_copyVector[j].getLatitude()); 
				if(aDistance <= safeDis)
				{
					Conflict aConflict(A_copyVector[j].getName(),aDistance,time, safeDis);
					conflictVector.push_back(aConflict);
					outConflictF << currentTime << " " << aConflict.getName() << " " << aConflict.getCPA() << " m " << aConflict.getTCPA() << " m " << aConflict.getPercentage() << "." << endl;
					A_vesselVector[j].setStatus(true);
					
					aConflict.outConflict(currentTime, outCPAF);
                         
					if(time == 0)
					{       
						bool update = false;           
						Collision aCollision(A_vesselVector[j].getName(), aDistance, currentTime, (safeDis - aDistance)/safeDis*1.0);
						for (int checkIndex = 0; update == false && checkIndex < A_collisionVector.size(); checkIndex++)
						{
						if (aCollision.getName() ==  A_collisionVector[checkIndex].getName())
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

    int findIndex;
    int searchIndex;
    if (!conflictVector.empty())
    {
         for (searchIndex = 0; searchIndex < conflictVector.size()&& conflictVector[searchIndex].getTCPA() <= 6; searchIndex++)
         {
           
           if (conflictVector[searchIndex].getTCPA() != 0)
           {                
            bool update = false;           
            Collision aCollision(conflictVector[searchIndex].getName(), conflictVector[searchIndex].getCPA(), conflictVector[searchIndex].getTCPA()+currentTime, (safeDis - conflictVector[searchIndex].getCPA())/safeDis*1.0);
                for (int checkIndex = 0; update == false && checkIndex < A_collisionVector.size(); checkIndex++)
                {
                 if (aCollision.getName() ==  A_collisionVector[checkIndex].getName())
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
         
         findIndex = searchIndex;
       
         if (findIndex < conflictVector.size()&& conflictVector[findIndex].getTCPA() != 0)
         {
          TCPA = conflictVector[findIndex].getTCPA();  
          cout << "Warning! The distance between " << A_vesselVector[0].getName() << " and " 
          << conflictVector[findIndex].getName() << " after " << TCPA << " minutes is: " 
          << conflictVector[findIndex].getCPA() << "m" << endl;
         
          return TCPA;   
         }
         else
         {
          cout << "Collision(s) recorded! Cannot be avoided!" << endl;
          return 0;   
         }       
    }
    else
    return -1;                         
}

bool checkFuture(vector<Vessel> A_vesselVector, vector<Collision> A_collisionVector, double safeDis,
	double conditionT, double mt, int currentTime,std::default_random_engine & gen,
	std::ofstream & outSpeedF, std::ofstream & outCourseF)
{
   int numReplications = 10;
   std::vector<int> collision;// = 0;
   std::vector<bool> check;
   check.resize(numReplications);
   collision.resize(numReplications);
   for (int replicate = 0; replicate < numReplications; replicate++)
   { 
	   check[replicate]=true;
   }
   Concurrency::parallel_for (int(0),int (numReplications),[&] (int replicate)
   //for (int replicate=0;replicate<numReplications;replicate++)
	{
	//check = true;
		vector<Vessel> copyVector;
	
		for (int i = 0; i < A_vesselVector.size(); i++)
		{
		 Vessel aVessel = A_vesselVector[i];
		 copyVector.push_back(aVessel);
		}
		//for (double time = 0; check == true && time <= conditionT; time += mt) //time in mins
		double time=0.0;
		while(time<= conditionT && check[replicate] == true)
		{
			  if (time != 0)
			  {
				  advanceRand(copyVector, mt, gen, outSpeedF, outCourseF);
			  }
        
			  for (int j = 1; check[replicate] == true && j < copyVector.size(); j++)
			  {
				double aDistance = getDistance (copyVector[0].getLongitude(), copyVector[0].getLatitude(), copyVector[j].getLongitude(), copyVector[j].getLatitude());   
				bool recorded = false;
				for (int search = 0; recorded == false && search < A_collisionVector.size(); search++)
				{
				 if (copyVector[j].getName() == A_collisionVector[search].getName())
				 recorded = true;
				}
				if(recorded == false && aDistance <= safeDis)
				{                 
				 check[replicate] = false;
				}  
			  }
			  time = time+mt;
		}  

		if (check[replicate] == false)
			collision[replicate]=1;
   }
   );
   int sum_of_elt = std::accumulate(collision.begin(),collision.end(),0);
   if ( sum_of_elt <= 1)
	   return true;
   else
	   return false;
   check.clear();
   collision.clear();
}

void advance (vector<Vessel>& A_vesselVector, double mt)
{
     
     for (int index = 0; index < A_vesselVector.size(); index++) //Model vessel movement in time intervals of 5 minutes
     { 
          
       double x_coordinate = A_vesselVector[index].getSpeed()*1.852/60*sin(toRadius(A_vesselVector[index].getCourse()))*mt;
       double y_coordinate = A_vesselVector[index].getSpeed()*1.852/60*cos(toRadius(A_vesselVector[index].getCourse()))*mt;
       double newLongitude = A_vesselVector[index].getLongitude() + 180/PI/cos(toRadius(A_vesselVector[index].getLatitude()))*x_coordinate/6371;
       double newLatitude = A_vesselVector[index].getLatitude() + 180/PI*y_coordinate/6371;
       A_vesselVector[index].setLongitude(newLongitude);
       A_vesselVector[index].setLatitude (newLatitude);
      
     }
     return;   
}

void checkConflict (vector<Vessel>& A_vesselVector, double safeDis,int runTime, double mt,
	std::ofstream & outCollisionF)
{
    vector<Collision> collisionVector;
	
	int countCollision;
    
    for (double time = 0; time <= runTime; time+=mt)
    {
		vector<Collision> tempcollisionVector;
		tempcollisionVector.resize(A_vesselVector.size());
		countCollision=0;
		Concurrency::parallel_for (int(1),int (A_vesselVector.size()),[&] (int j)
        //for (int j = 1; j < A_vesselVector.size(); j++)
        {
            
          double aDistance = getDistance (A_vesselVector[0].getLongitude(), A_vesselVector[0].getLatitude(), A_vesselVector[j].getLongitude(), A_vesselVector[j].getLatitude()); 
          
          if(aDistance <= safeDis && A_vesselVector[j].getStatus() == false)
          {
           Collision aCollision(A_vesselVector[j].getName(),aDistance,time, (safeDis - aDistance)/safeDis);
           //collisionVector.push_back(aCollision);
		   tempcollisionVector[j]=aCollision;
		   aCollision.outCollision(outCollisionF);
           A_vesselVector[j].setStatus(true); 
		   countCollision++;
          }
        });
        advance (A_vesselVector, mt);
		if (countCollision>0)
		{
			for(int jj=0;jj<countCollision;jj++)
				collisionVector.push_back(tempcollisionVector[jj]);
		}
		tempcollisionVector.clear();
    }       
}

bool changeSpeed(vector<Vessel>& A_vesselVector, vector<Collision> A_collisionVector, double aSafeDis,
	double conditionT, double mt, int currentTime,std::default_random_engine & gen,
	std::ofstream & outSpeedF, std::ofstream & outCourseF,std::ofstream & outManeuvreF)
{
 double storeCourse = A_vesselVector[0].getCourse();
 double storeSpeed = A_vesselVector[0].getSpeed();
 for (int schange = -2; schange >= -10 && A_vesselVector[0].getSpeed() > 1; schange--)
    { 
     for (int cchange = 15; cchange <= 60; cchange++)
     {
      A_vesselVector[0].setSpeed(storeSpeed + schange);
      A_vesselVector[0].setCourse(storeCourse + cchange);     
      if (checkFuture(A_vesselVector, A_collisionVector, aSafeDis, conditionT, mt, 
		  currentTime,gen,outSpeedF,outCourseF)) 
      {
       cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
       cout << "Followed by a speed change of " << schange << " knots" << endl;
       Manoeuvre aManoeuvre(currentTime, cchange, schange);
	   aManoeuvre.outManoeuvre(outManeuvreF);
       return true;
      }    
     }   
     for (int cchange = -15; cchange >= -60; cchange--)
     {    
      A_vesselVector[0].setSpeed(storeSpeed + schange);
      A_vesselVector[0].setCourse(storeCourse + cchange);     
      if (checkFuture(A_vesselVector, A_collisionVector, aSafeDis, conditionT, mt,
		  currentTime, gen, outSpeedF, outCourseF))
      {
       cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
       cout << "Followed by a speed change of " << schange << " knots" << endl;
       Manoeuvre aManoeuvre(currentTime, cchange, schange);
	   aManoeuvre.outManoeuvre(outManeuvreF);
       return true;
      }    
     }  
    } 
    
 for (int schange = 2; schange <= 10 ; schange++)
    {
     for (int cchange = 15; cchange <= 60; cchange++)
     {   
      A_vesselVector[0].setSpeed(storeSpeed + schange);
      A_vesselVector[0].setCourse(storeCourse + cchange);     
      if (checkFuture(A_vesselVector, A_collisionVector, aSafeDis, conditionT, mt,
		  currentTime, gen, outSpeedF, outCourseF))
      {
       cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
       cout << "Followed by a speed change of " << schange << " knots" << endl;
       Manoeuvre aManoeuvre(currentTime, cchange, schange);
	   aManoeuvre.outManoeuvre(outManeuvreF);
       return true;
      }    
     }   
     for (int cchange = -15; cchange >= -60; cchange--)
     {     
      A_vesselVector[0].setSpeed(storeSpeed + schange);
      A_vesselVector[0].setCourse(storeCourse + cchange);     
      if (checkFuture(A_vesselVector, A_collisionVector, aSafeDis, conditionT, mt,
		  currentTime, gen, outSpeedF, outCourseF))
      {
       cout << "Conflict can be avoided by a course change of " << cchange << " degrees" << endl;
       cout << "Followed by a speed change of " << schange << " knots" << endl;
       Manoeuvre aManoeuvre(currentTime, cchange, schange);
	   aManoeuvre.outManoeuvre(outManeuvreF);
       return true;
      }    
     }  
    } 
 A_vesselVector[0].setCourse(storeCourse);  
 A_vesselVector[0].setSpeed(storeSpeed);    
 return false; 
}
bool changeCourse(vector<Vessel>& A_vesselVector, vector<Collision> A_collisionVector, double aSafeDis,
	double conditionT, double mt, int currentTime,std::default_random_engine & gen,
	std::ofstream & outSpeedF, std::ofstream & outCourseF, std::ofstream & outManeuvreF)
{
 
 double storeCourse = A_vesselVector[0].getCourse();
 for (int change = 15; change <= 60; change++)
    {
  
     A_vesselVector[0].setCourse(storeCourse + change);     
     if (checkFuture(A_vesselVector, A_collisionVector, aSafeDis, conditionT, mt,
		 currentTime, gen, outSpeedF, outCourseF))
     {
      cout << "Conflict can be avoided by a course change of " << change << " degrees" << endl;
      Manoeuvre aManoeuvre(currentTime, change, 0);
	  aManoeuvre.outManoeuvre(outManeuvreF);
      return true;
     }    
    }
  A_vesselVector[0].setCourse(storeCourse);
  for (int change = -15; change >= -60; change--)
    {
      
     A_vesselVector[0].setCourse(storeCourse + change);     
     if (checkFuture(A_vesselVector,A_collisionVector, aSafeDis, conditionT, mt,
		 currentTime, gen, outSpeedF, outCourseF))
     {
      cout << "Conflict can be avoided by a course change of " << change << " degrees" << endl;
      Manoeuvre aManoeuvre(currentTime, change, 0);
	  aManoeuvre.outManoeuvre(outManeuvreF);
      return true;
     }    
    }  
 A_vesselVector[0].setCourse(storeCourse);   
 if (changeSpeed(A_vesselVector, A_collisionVector, aSafeDis, conditionT, mt,
	 currentTime, gen, outSpeedF, outCourseF, outManeuvreF))
 return true;
 else
 return false;              
}



int main(int argc, char *argv[])
{
    long int seedNum=12345;
	std::default_random_engine generator(12345); //the seed helps to repeat the experiment
	
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

			std::ifstream readFile(fileTestName.c_str());

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
			/*clearFile.open("Collision.txt");
			clearFile.clear();
			clearFile.close();
			clearFile.open("CPA.txt");
			clearFile.clear();
			clearFile.close();
			clearFile.open("Manoeuvre.txt");
			clearFile.clear();
			clearFile.close();
			clearFile.open("Infor.txt");
			clearFile.clear();
			clearFile.close();
			clearFile.open("Check.txt");
			clearFile.clear();
			clearFile.close();*/

			string read; //Temp string to read line from txt
			vector<Vessel> vesselVector;
			vector<Collision> collisionVector;
			double safeDis = 0;
			int runTime = 180;
			double MT = 30; // minitor time in minutes
			double conditionT;
			double mt = 1; //increment time in minutes
			int numIndex = 0;
			double safeFactor = 5.5;
			double checkSafeDis;
			vector<double> numVector;
			while (getline(readFile, read)) //Read file
			{
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
			}

			bool randReaction = true;
			bool manoeuvre = true;

			safeDis = vesselVector[0].getLength() * 4;
			checkSafeDis = vesselVector[0].getLength()*safeFactor;

			if (manoeuvre == true)
			{
				for (double loopTime = 0; loopTime <= runTime; loopTime += mt) //Simulate for 2 hours
				{
					std::cout << "Current Real Time: " << loopTime << endl;
					vector<Vessel> copyVector;
					cout << "At time t = " << loopTime << "m" << endl;
					double ETC; //Earliest time to collision
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
					ETC = findTime(vesselVector, copyVector, collisionVector, safeDis, MT, mt, loopTime, outConflictF,
						outCPA, outCollision);
					string option = "Y";

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
						advanceRand(vesselVector, mt, generator, outSpeedF, outCourseF);
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
								bool feasible;
								feasible = changeCourse(vesselVector, collisionVector, checkSafeDis, conditionT, mt,
									loopTime, generator, outSpeedF, outCourseF, outManoeuvreF);
								while (!feasible && safeFactor > 4.0)
								{
									safeFactor -= 0.5;
									//std::cout << "Current Safety Factor: " << safeFactor << endl;
									checkSafeDis = vesselVector[0].getLength()*safeFactor;
									feasible = changeCourse(vesselVector, collisionVector, checkSafeDis, conditionT, mt, loopTime,
										generator, outSpeedF, outCourseF, outManoeuvreF);

								}
								advanceRand(vesselVector, mt, generator, outSpeedF, outCourseF);
							}
							else
							{
								cout << "Case 3: Collision detected, solution ignored." << endl;
								advanceRand(vesselVector, mt, generator, outSpeedF, outCourseF);
							}
						}
						else
						{
							cout << "Case 5: No collision detected." << endl;
							advanceRand(vesselVector, mt, generator, outSpeedF, outCourseF);
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
				checkConflict(vesselVector, safeDis, runTime, mt, outCollision);
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
    
