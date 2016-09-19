#include "Manoeuvre.h"
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

//ofstream manoeuvreFile ("Manoeuvre.txt", ios::app);

Manoeuvre :: Manoeuvre (int aTime, double aCourse, double aSpeed)
{
  Time = aTime;
  Coursechange = aCourse;
  Speedchange = aSpeed;
}

int Manoeuvre :: getTime (){return Time;}
double Manoeuvre :: getCourse () {return Coursechange;}
double Manoeuvre :: getSpeed(){return Speedchange;}

void Manoeuvre :: printManoeuvre ()
{
  cout << Time << " " << Coursechange << " " << Speedchange << endl; 
}

void Manoeuvre::outManoeuvre(std::ofstream & manoeuvreFile)
{
  manoeuvreFile << Time << " " << Coursechange << " " << Speedchange << endl; 
}
