#include "Randcourse.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using std::string;

//ofstream courseFile ("Randcourse.txt", ios::app);

Randcourse :: Randcourse (string aName, double aOriginal, double aChange, double aNew)
{
  Vessel_Name = aName;
  Original = aOriginal;
  Change = aChange;
  New = aNew;
}
    
string Randcourse :: getName (){return Vessel_Name;}
double Randcourse :: getOriginal (){return Original;}
double Randcourse :: getChange () {return Change;}
double Randcourse :: getNew(){return New;}
void Randcourse :: printRandcourse ()
{
  cout << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}
void Randcourse::outRandcourse(std::ofstream & courseFile)
{
  courseFile << Vessel_Name << " " << Original << " " << Change << " " << New << endl;
}