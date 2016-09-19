#include "Conflict.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using std::string;

//ofstream conflictFile ("CPA.txt", ios::app);

Conflict :: Conflict (string aName, double aCPA, double aTCPA, double aSafeDis)
{
  Vessel_Name = aName;
  CPA = aCPA;
  TCPA = aTCPA;
  SafeDis = aSafeDis;
  Percentage = (SafeDis - CPA)/SafeDis*1.00;
}
    
string Conflict :: getName (){return Vessel_Name;}
double Conflict :: getCPA (){return CPA;}
double Conflict :: getTCPA () {return TCPA;}
double Conflict :: getSafeDis(){return SafeDis;}
double Conflict :: getPercentage(){return Percentage;}
void Conflict::outConflict(int currentTime, std::ofstream & conflictFile)
{
  //cout << "Vessel " << Vessel_Name << " violates own vessel's ship domain with closet distance of " << CPA << "m " << "in "<< TCPA << " min." << endl;
  //cout << "Degree of violation is " << Percentage  << "." << endl; 
  conflictFile << currentTime << " " << Vessel_Name << " " << CPA << "m " << TCPA << "min " << Percentage  << "." << endl; 
}
