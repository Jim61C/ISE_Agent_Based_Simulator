#include <iostream>
#include <string>

using namespace std;
using std::string;

#ifndef COLLISION_H
#define COLLISION_H

class Collision {

    private:
    string Vessel_Name;
    double CPA; 
    double Time;
    double Percentage;
    
    public:
	Collision();
    Collision (string, double, double, double); //Constructor
    string getName ();
    double getCPA ();
    double getTime ();
    double getPercentage ();
    void setCPA (double);
    void setTime (double);
    void setPercentage (double);
    void printCollision();
    void outCollision(std::ofstream & outFile);
};
#endif