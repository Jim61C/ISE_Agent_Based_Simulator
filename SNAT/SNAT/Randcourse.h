#include <iostream>
#include <string>

using namespace std;
using std::string;

#ifndef RANDCOUSR_H
#define RANDCOURSE_H

class Randcourse {

    private:
    string Vessel_Name;
    double Original;
    double Change;
    double New;
    
    public:
    Randcourse (string, double, double, double); //Constructor
    string getName ();
    double getOriginal ();
    double getChange ();
    double getNew();
    void printRandcourse();
    void outRandcourse (std::ofstream & outRandCourse);
    
};
#endif