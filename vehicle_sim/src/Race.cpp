// Race.cpp
#include "Race.h"
#include <algorithm>
#include <cmath>

namespace { constexpr double MPS_TO_MPH = 2.23694; }

Race::Race(double dt) : dt_(dt) {}

RaceResults Race::runFixedDistance(std::vector<Vehicle*>& vehicles,
                                    double distanceM) {
    RaceResults results;
    results.distanceM = distanceM;
    results.entries.reserve(vehicles.size());
    results.traces.resize(vehicles.size());

    // Reset all vehicles and set up per-vehicle state.
    std::vector<bool> finished(vehicles.size(), false);
    std::vector<double> finishTimes(vehicles.size(), -1.0);
    std::vector<double> finishSpeeds(vehicles.size(), 0.0);

    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        vehicles[i]->reset();
        results.traces[i].vehicleName = vehicles[i]->name();
    }

    double t = 0.0;
    const double maxTime = 120.0;  // safety cap

    while (t <= maxTime) {
        bool allDone = true;
        for (std::size_t i = 0; i < vehicles.size(); ++i) {
            if (finished[i]) continue;
            allDone = false;

            auto step = vehicles[i]->step(dt_);

            TraceSample s;
            s.time = t;
            s.position = vehicles[i]->position();
            s.speedMps = vehicles[i]->speed();
            s.speedMph = s.speedMps * MPS_TO_MPH;
            s.rpm = vehicles[i]->rpm();
            s.gear = vehicles[i]->transmission().currentGear();
            s.engineTorqueNm = step.engineTorqueNm;
            s.wheelForceN    = step.wheelForceN;
            s.dragN          = step.dragN;
            s.rollingN       = step.rollingN;
            s.accelMps2      = step.accelMps2;
            results.traces[i].trace.push_back(s);

            if (s.position >= distanceM) {
                finished[i] = true;
                finishTimes[i] = t;
                finishSpeeds[i] = s.speedMph;
            }
        }
        if (allDone) break;
        t += dt_;
    }

    // Build entries, unfinished cars get their last recorded position.
    for (std::size_t i = 0; i < vehicles.size(); ++i) {
        RaceEntry e;
        e.vehicleName = vehicles[i]->name();
        e.finishTime = (finishTimes[i] >= 0.0) ? finishTimes[i] : maxTime;
        e.finishSpeedMph = finishSpeeds[i];
        e.positionAtEnd = vehicles[i]->position();
        results.entries.push_back(e);
    }

    // Sort by finish time (fastest first).
    std::sort(results.entries.begin(), results.entries.end(),
        [](const RaceEntry& a, const RaceEntry& b) {
            return a.finishTime < b.finishTime;
        });

    return results;
}
