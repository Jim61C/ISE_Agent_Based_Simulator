#pragma once
class TrajectoryPoint
{
private:
	double lat;
	double lon;
	double rate_of_turn;
	double course_over_ground;
	double true_heading;
	double speed_over_ground;
	long mmsi; // optional mmsi id of the vessel that this 'endpoint' correspond to, 0 indicates field not set

public:
	TrajectoryPoint(); // default constructor
	TrajectoryPoint(double lat, double lon); // for endpoints, where the other info are redundant
	TrajectoryPoint(double lat, double lon, double rate_of_turn, 
		double course_over_ground, double true_heading, double speed_over_ground);
	TrajectoryPoint(double lat, double lon, double rate_of_turn, 
		double course_over_ground, double true_heading, double speed_over_ground, long mmsi);
	bool operator <(const TrajectoryPoint& rhs) const; // used for set and map

	~TrajectoryPoint();
	double getLatitude() const;
	void setLatitude(double lat);

	double getLongitude() const;
	void setLongitude(double lon);

	double getRateOfTurn() const;
	void setRateOfTurn(double rate_of_turn);

	double getCourseOverGround() const;
	void setCourseOverGround(double course_over_ground);

	double getTrueHeading() const;
	void setTrueHeading(double true_heading);

	double getSpeedOverGround() const;
	void setSpeedOverGround(double spped_over_ground);

	long getMMSI() const;
	void setMMSI(long mmsi);

	void print();
};

