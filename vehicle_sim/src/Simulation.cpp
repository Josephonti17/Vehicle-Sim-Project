// Simulation.cpp
#include "Simulation.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>

namespace {
    constexpr double MPS_TO_MPH = 2.23694;
    constexpr double QUARTER_MILE_M = 402.336;  // 1/4 mile in meters
}

Simulation::Simulation(double dt) : dt_(dt) {}

SimulationResults Simulation::run(Vehicle& vehicle,
                                   TestType test,
                                   double durationSeconds) {
    SimulationResults r;
    r.vehicleName = vehicle.name();
    r.test = test;

    vehicle.reset();

    // Stopping conditions depend on the test type.
    double stopTime = durationSeconds;
    bool   runUntilTopSpeed = false;

    switch (test) {
        case TestType::ZeroToSixty:  stopTime = 30.0; break;
        case TestType::QuarterMile:  stopTime = 60.0; break;
        case TestType::TopSpeed:
            runUntilTopSpeed = true;
            stopTime = 300.0;   // safety cap
            break;
        case TestType::FixedDuration: break; // stopTime already set
    }

    double t = 0.0;
    double prevSpeed = 0.0;
    double stagnationTime = 0.0;
    const double stagnationLimit = 2.0;  // seconds

    while (t <= stopTime) {
        auto step = vehicle.step(dt_);

        TraceSample s;
        s.time           = t;
        s.position       = vehicle.position();
        s.speedMps       = vehicle.speed();
        s.speedMph       = vehicle.speed() * MPS_TO_MPH;
        s.rpm            = vehicle.rpm();
        s.gear           = vehicle.transmission().currentGear();
        s.engineTorqueNm = step.engineTorqueNm;
        s.wheelForceN    = step.wheelForceN;
        s.dragN          = step.dragN;
        s.rollingN       = step.rollingN;
        s.accelMps2      = step.accelMps2;
        r.trace.push_back(s);

        // Record milestone times.
        if (r.zeroToThirtyTime < 0.0 && s.speedMph >= 30.0)
            r.zeroToThirtyTime = t;
        if (r.zeroToSixtyTime < 0.0 && s.speedMph >= 60.0)
            r.zeroToSixtyTime = t;
        if (r.zeroToHundredTime < 0.0 && s.speedMph >= 100.0)
            r.zeroToHundredTime = t;
        if (r.quarterMileTime < 0.0 && s.position >= QUARTER_MILE_M) {
            r.quarterMileTime   = t;
            r.quarterMileTrapMph = s.speedMph;
        }

        // Per-test stopping logic.
        if (test == TestType::ZeroToSixty && r.zeroToSixtyTime >= 0.0) break;
        if (test == TestType::QuarterMile && r.quarterMileTime >= 0.0) break;
        if (runUntilTopSpeed) {
            // Detect when acceleration drops near zero (top speed).
            if (std::abs(vehicle.speed() - prevSpeed) / dt_ < 0.01 &&
                vehicle.speed() > 1.0) {
                stagnationTime += dt_;
                if (stagnationTime >= stagnationLimit) break;
            } else {
                stagnationTime = 0.0;
            }
        }
        prevSpeed = vehicle.speed();
        t += dt_;
    }

    r.totalTime = t;
    r.totalDistance = vehicle.position();

    // Top speed = max mph in the trace.
    if (!r.trace.empty()) {
        auto it = std::max_element(r.trace.begin(), r.trace.end(),
            [](const TraceSample& a, const TraceSample& b) {
                return a.speedMph < b.speedMph;
            });
        r.topSpeedMph = it->speedMph;
    }

    return r;
}

bool Simulation::writeTraceCsv(const SimulationResults& results,
                                const std::string& filename) {
    std::ofstream f(filename);
    if (!f) return false;

    f << "time_s,position_m,speed_mps,speed_mph,rpm,gear,"
         "engine_torque_nm,wheel_force_n,drag_n,rolling_n,accel_mps2\n";
    f << std::fixed << std::setprecision(4);
    for (const auto& s : results.trace) {
        f << s.time << ','
          << s.position << ','
          << s.speedMps << ','
          << s.speedMph << ','
          << s.rpm << ','
          << s.gear + 1 << ','  // human 1-indexed
          << s.engineTorqueNm << ','
          << s.wheelForceN << ','
          << s.dragN << ','
          << s.rollingN << ','
          << s.accelMps2 << '\n';
    }
    return true;
}
