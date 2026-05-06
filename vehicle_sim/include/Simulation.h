// Simulation.h
// Runs a single Vehicle through one of several standard tests
// (0-60 mph, quarter mile, top speed) and collects a time-series trace.
#ifndef SIMULATION_H
#define SIMULATION_H

#include "Vehicle.h"
#include <vector>
#include <string>

enum class TestType {
    ZeroToSixty,
    QuarterMile,
    TopSpeed,
    FixedDuration
};

// One sample of the simulation trace.
struct TraceSample {
    double time;      // s
    double position;  // m
    double speedMps;  // m/s
    double speedMph;  // mph
    double rpm;
    std::size_t gear;
    double engineTorqueNm;
    double wheelForceN;
    double dragN;
    double rollingN;
    double accelMps2;
};

struct SimulationResults {
    std::string vehicleName;
    TestType test;
    double zeroToSixtyTime = -1.0; // s, -1 if not reached
    double zeroToThirtyTime = -1.0;
    double zeroToHundredTime = -1.0;
    double quarterMileTime = -1.0;
    double quarterMileTrapMph = -1.0;
    double topSpeedMph = -1.0;
    double totalTime = 0.0;
    double totalDistance = 0.0;
    std::vector<TraceSample> trace;
};

class Simulation {
public:
    Simulation(double dt = 0.001);

    // Runs the given vehicle. The vehicle is reset before the run.
    // For FixedDuration, specify durationSeconds.
    SimulationResults run(Vehicle& vehicle,
                          TestType test,
                          double durationSeconds = 30.0);

    // Write a trace to a CSV file. Returns true on success.
    static bool writeTraceCsv(const SimulationResults& results,
                              const std::string& filename);

    double dt() const { return dt_; }
    void setDt(double v) { dt_ = v; }

private:
    double dt_;
};

#endif // SIMULATION_H
