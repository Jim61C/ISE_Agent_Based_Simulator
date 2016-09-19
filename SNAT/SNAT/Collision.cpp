#include "Collision.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using std::string;

//ofstream collisionFile ("Collision.txt", ios::app);

Collision :: Collision (string aName, double aCPA, double aTime, double aPercentage)
{
  Vessel_Name = aName;
  CPA = aCPA;
  Time = aTime;
  Percentage = aPercentage;
}
Collision :: Collision ()
{}
    
string Collision :: getName (){return Vessel_Name;}
double Collision :: getCPA (){return CPA;}
double Collision :: getTime (){return Time;}
double Collision :: getPercentage (){return Percentage;}
void Collision :: setCPA (double newCPA){CPA = newCPA;}
void Collision :: setTime (double newTime){Time = newTime;}
void Collision :: setPercentage (double newPercentage){Percentage = newPercentage;}
void Collision :: printCollision ()
{
  cout << "Vessel " << Vessel_Name << " violates own vessel's ship domain with closet distance of " << CPA << endl;
}
void Collision :: outCollision (std::ofstream & collisionFile)
{
  collisionFile << Time << " " << Vessel_Name << " " << CPA << " " << Percentage << " " << endl;
}
