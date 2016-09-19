#include <iostream>
#include <string>

using namespace std;
using std::string;

#ifndef RANDSPEED_H
#define RANDSPEED_H

class Randspeed {

    private:
    string Vessel_Name;
    double Original;
    double Change;
    double New;
    
    public:
    Randspeed (string, double, double, double); //Constructor
    string getName ();
    double getOriginal ();
    double getChange ();
    double getNew();
    void printRandspeed();
    void outRandspeed (std::ofstream & outRandSpeedF);
    
};
#endif
