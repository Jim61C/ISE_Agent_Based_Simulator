#pragma once
#include "TrajectoryLoader.h"
#include "Trajectory.h"
#include "Constants.h"

class UnitTester
{
public:
	UnitTester();
	~UnitTester();
	void unitTestLoadMMSIIds(TrajectoryLoader loader);
	void unitTestLoadMINDistance(TrajectoryLoader loader);
	void unitTestLoadEndpoints(TrajectoryLoader loader);
	void unitTestLoadTrajectories(TrajectoryLoader loader);
	void unitTestVesselAddingRemovingWaitingVesselNames();
	void unitTestGetIntersection();
};

