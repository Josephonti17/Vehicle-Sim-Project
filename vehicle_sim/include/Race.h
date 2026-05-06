// Race.h
// Runs multiple vehicles in lockstep and reports head-to-head results.
#ifndef RACE_H
#define RACE_H

#include "Vehicle.h"
#include "Simulation.h"
#include <vector>
#include <string>

struct RaceEntry {
    std::string vehicleName;
    double finishTime;    // s
    double finishSpeedMph;
    double positionAtEnd; // m
};

struct RaceResults {
    double distanceM;
    std::vector<RaceEntry> entries;  // sorted by finishTime (winner first)
    std::vector<SimulationResults> traces;
};

class Race {
public:
    explicit Race(double dt = 0.001);

    // Run all vehicles over a fixed distance (default quarter mile).
    RaceResults runFixedDistance(std::vector<Vehicle*>& vehicles,
                                 double distanceM = 402.336);

private:
    double dt_;
};

#endif // RACE_H
