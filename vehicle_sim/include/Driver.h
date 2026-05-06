// Driver.h
// Abstract base class representing a shift strategy, with concrete derived
// classes implementing different driving styles. This is the polymorphism
// requirement for the OOP section of the project.
#ifndef DRIVER_H
#define DRIVER_H

#include <string>

class Vehicle;  // forward declaration to avoid circular include

// State passed to the driver each time step so it can make a shift decision.
struct DriverState {
    double speedMps;
    double engineRpm;
    double engineTorqueNm;
    double wheelForceN;
    std::size_t currentGear;
};

// Possible actions the driver may request.
enum class ShiftAction {
    None,
    Up,
    Down
};

class Driver {
public:
    virtual ~Driver() = default;

    // Each derived class decides what to do based on engine RPM, current gear,
    // and the vehicle's characteristics (needed for "optimal" strategies).
    virtual ShiftAction shiftDecision(const DriverState& state,
                                      const Vehicle& vehicle) const = 0;

    virtual std::string name() const = 0;
};

// Shifts up at redline and down at a low RPM threshold. Maximum aggression.
class AggressiveDriver : public Driver {
public:
    explicit AggressiveDriver(double shiftUpRpm = 6300.0,
                              double shiftDownRpm = 2500.0);

    ShiftAction shiftDecision(const DriverState& state,
                              const Vehicle& vehicle) const override;

    std::string name() const override { return "Aggressive"; }

private:
    double shiftUpRpm_;
    double shiftDownRpm_;
};

// Picks the gear that maximizes wheel force at the current speed, by
// checking the projected RPM in each gear and looking up torque. This is
// closer to what an actual dragstrip driver would do with a close-ratio box.
class OptimalDriver : public Driver {
public:
    OptimalDriver() = default;

    ShiftAction shiftDecision(const DriverState& state,
                              const Vehicle& vehicle) const override;

    std::string name() const override { return "Optimal"; }
};

#endif // DRIVER_H
