// Vehicle.cpp
#include "Vehicle.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <algorithm>
#include <cmath>
#include <utility>

namespace {
    constexpr double G          = 9.81;   // gravity, m/s^2
    constexpr double AIR_DENSITY = 1.225; // kg/m^3 at sea level
}

Vehicle::Vehicle()
    : name_("Unnamed"),
      mass_(1500.0),
      cd_(0.32),
      area_(2.2),
      speed_(0.0),
      pos_(0.0) {}

Vehicle::Vehicle(const std::string& name,
                 double massKg,
                 double dragCoefficient,
                 double frontalAreaM2,
                 Engine engine,
                 Transmission transmission,
                 Tire tire,
                 std::unique_ptr<Driver> driver)
    : name_(name),
      mass_(massKg),
      cd_(dragCoefficient),
      area_(frontalAreaM2),
      engine_(std::move(engine)),
      trans_(std::move(transmission)),
      tire_(std::move(tire)),
      driver_(std::move(driver)),
      speed_(0.0),
      pos_(0.0) {}

double Vehicle::rpm() const {
    double ratio = trans_.totalRatio(trans_.currentGear());
    double rpm   = speed_ / tire_.radius() * ratio * 60.0 / (2.0 * M_PI);
    if (rpm < engine_.idleRpm()) rpm = engine_.idleRpm();
    return rpm;
}

void Vehicle::reset() {
    speed_ = 0.0;
    pos_   = 0.0;
    trans_.setCurrentGear(0);
}

Vehicle::StepResult Vehicle::step(double dt) {
    StepResult r{};
    r.gearBefore = trans_.currentGear();

    // 1. Engine RPM from road speed and gear.
    double currentRpm = rpm();

    // 2. Ask the driver whether to shift before computing force.
    DriverState dstate{speed_, currentRpm,
                       engine_.torqueAt(currentRpm),
                       0.0, trans_.currentGear()};
    ShiftAction action = driver_->shiftDecision(dstate, *this);
    if      (action == ShiftAction::Up)   trans_.shiftUp();
    else if (action == ShiftAction::Down) trans_.shiftDown();

    // 3. Recompute RPM and torque in the (possibly new) gear.
    currentRpm = rpm();
    double engineTorque = engine_.torqueAt(currentRpm);
    double ratio = trans_.totalRatio(trans_.currentGear());
    double wheelTorque = engineTorque * ratio * trans_.efficiency();
    double wheelForce  = wheelTorque / tire_.radius();

    // 4. Traction limit: cannot exceed mu * m * g.
    double tractionLimit = tire_.grip() * mass_ * G;
    if (wheelForce > tractionLimit) wheelForce = tractionLimit;

    // 5. Resistive forces.
    double drag = 0.5 * AIR_DENSITY * cd_ * area_ * speed_ * speed_;
    double rolling = tire_.rollingResistance() * mass_ * G;
    if (speed_ < 0.01) rolling = 0.0;  // no rolling resistance at a stop

    double netForce = wheelForce - drag - rolling;
    double accel    = netForce / mass_;

    // 6. Euler integration.
    speed_ += accel * dt;
    if (speed_ < 0.0) speed_ = 0.0;
    pos_   += speed_ * dt;

    r.engineTorqueNm = engineTorque;
    r.wheelForceN    = wheelForce;
    r.dragN          = drag;
    r.rollingN       = rolling;
    r.netForceN      = netForce;
    r.accelMps2      = accel;
    r.gearAfter      = trans_.currentGear();
    return r;
}
