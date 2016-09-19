#include <iostream>
#include <string>
#include <random>
using namespace std;
using std::string;

#ifndef GEOMETRY_H
#define GEOMETRY_H
#define PI 3.14159

namespace GeometryMethods{
	inline double toRadius(double degree) //Unit conversion from degree to radius
	{
		return degree * PI / 180.0;
	}
	inline double randGenerator(std::default_random_engine & gen) // will return [0, 1)
	{

		//std::default_random_engine generator(12345); //the seed helps to repeat the experiment
		std::uniform_real_distribution<double> distribution(0, 1);
		//double aRand = distribution(generator);
		double aRand = distribution(gen);
		return aRand;
	}
	inline bool randSpeed(std::default_random_engine & gen)
	{
		if (randGenerator(gen) <= 0.12) //There will be a random change in speed
			return true;
		else //There will not be a random change in speed
			return false;

	}
	inline bool randCourse(std::default_random_engine & gen)
	{
		if (randGenerator(gen) <= 0.37) //There will be a random change in course
			return true;
		else //There will not be a random change in course
			return false;
	}
	inline double normGenerator(std::default_random_engine & gen) //Use two random numbers to generate a random number that follows the standard normal distribution
	{
		double normRand = sqrt(-2 * 2.303*log(randGenerator(gen)))*cos(2 * PI*randGenerator(gen));
		return normRand;
	}
	inline double lognormGenerator(double mean, double std, std::default_random_engine & gen)
	{
		double normRand = normGenerator(gen);
		double lognormRand = exp(mean + std*normRand);
		return lognormRand;
	}
}
#endif