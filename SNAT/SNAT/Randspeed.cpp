#include "Randspeed.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using std::string;

//ofstream speedFile ("Randspeed.txt", ios::app);

Randspeed :: Randspeed (string aName, double aOriginal, double aChange, double aNew)
{
  Vessel_Name = aName;
  Original = aOriginal;
  Change = aChange;
  New = aNew;
}
    
string Randspeed :: getName (){return Vessel_Name;}
double Randspeed :: getOriginal (){return Original;}
double Randspeed :: getChange () {return Change;}
double Randspeed :: getNew(){return New;}
void Randspeed :: printRandspeed ()
{
  cout << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}
void Randspeed::outRandspeed(std::ofstream & speedFile)
{
  speedFile << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}
