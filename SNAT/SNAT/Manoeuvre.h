#include <iostream>
#include <string>

using namespace std;

#ifndef MANOEUVRE_H
#define MANOEUVRE_H

class Manoeuvre {

    private:
    int Time ; 
    double Coursechange;
    double Speedchange;
    
    public:
    Manoeuvre (int, double, double); //Constructor
    int getTime ();
    double getCourse ();
    double getSpeed();
    void printManoeuvre ();
    void outManoeuvre (std::ofstream & outManeuvreF);
    
};
#endif
