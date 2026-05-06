// Vehicle.h
// A Vehicle composes an Engine, Transmission, and Tire, plus mass and
// aerodynamic properties. It owns a polymorphic Driver through a
// std::unique_ptr, giving each Vehicle a clearly scoped shift strategy.
//
// The Vehicle provides the physics step used by Simulation.
#ifndef VEHICLE_H
#define VEHICLE_H

#include "Engine.h"
#include "Transmission.h"
#include "Tire.h"
#include "Driver.h"

#include <memory>
#include <string>

class Vehicle {
public:
    Vehicle();
    Vehicle(const std::string& name,
            double massKg,
            double dragCoefficient,
            double frontalAreaM2,
            Engine engine,
            Transmission transmission,
            Tire tire,
            std::unique_ptr<Driver> driver);

    // Move-only (because of unique_ptr). Copy control is deleted explicitly.
    Vehicle(const Vehicle&) = delete;
    Vehicle& operator=(const Vehicle&) = delete;
    Vehicle(Vehicle&&) noexcept = default;
    Vehicle& operator=(Vehicle&&) noexcept = default;

    // Accessors
    const std::string& name() const        { return name_; }
    double mass() const                    { return mass_; }
    double cd() const                      { return cd_; }
    double frontalArea() const             { return area_; }

    const Engine& engine() const           { return engine_; }
    const Transmission& transmission() const { return trans_; }
    Transmission& transmission()           { return trans_; }
    const Tire& tire() const               { return tire_; }
    const Driver& driver() const           { return *driver_; }

    // Current dynamic state
    double speed() const  { return speed_; }
    double position() const { return pos_; }
    double rpm() const;

    void setSpeed(double v)    { speed_ = v; }
    void setPosition(double x) { pos_ = x; }
    void reset();

    // One Euler integration step of duration dt seconds.
    // Returns the net force applied (for diagnostics / CSV output).
    struct StepResult {
        double engineTorqueNm;
        double wheelForceN;    // after traction limit
        double dragN;
        double rollingN;
        double netForceN;
        double accelMps2;
        std::size_t gearBefore;
        std::size_t gearAfter;
    };
    StepResult step(double dt);

    // Change the driver at runtime (demonstrates unique_ptr ownership transfer).
    void setDriver(std::unique_ptr<Driver> d) { driver_ = std::move(d); }

private:
    std::string name_;
    double mass_;   // kg
    double cd_;     // drag coefficient
    double area_;   // frontal area, m^2

    Engine engine_;
    Transmission trans_;
    Tire tire_;

    std::unique_ptr<Driver> driver_;

    // Dynamic state
    double speed_;   // m/s
    double pos_;     // m
};

#endif // VEHICLE_H
