#include <iostream>
#include <string>

using namespace std;
using std::string;

#ifndef CONFLICT_H
#define CONFLICT_H

class Conflict {

    private:
    string Vessel_Name;
    double CPA; 
    double TCPA;
    double SafeDis;
    double Percentage;
    
    public:
    Conflict (string, double, double, double); //Constructor
    string getName ();
    double getCPA ();
    double getTCPA ();
    double getSafeDis();
    double getPercentage();
	void outConflict(int time, std::ofstream & conflictFile);
    
};
#endif
