// Transmission.h
// Models a multi-speed transmission with fixed gear ratios, a final drive,
// and a drivetrain efficiency factor. The current gear is part of the state.
#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include <vector>
#include <cstddef>

class Transmission {
public:
    Transmission();
    Transmission(const std::vector<double>& gearRatios,
                 double finalDrive,
                 double efficiency = 0.85);

    void setGearRatios(const std::vector<double>& ratios) { gearRatios_ = ratios; }
    void setFinalDrive(double fd)   { finalDrive_ = fd; }
    void setEfficiency(double e)    { efficiency_ = e; }

    const std::vector<double>& gearRatios() const { return gearRatios_; }
    double finalDrive() const { return finalDrive_; }
    double efficiency() const { return efficiency_; }

    // Total ratio from engine to wheel for the given gear index (0-based).
    double totalRatio(std::size_t gearIdx) const;

    std::size_t numGears() const { return gearRatios_.size(); }

    // Current gear state (0-based). Defaults to 1st gear (index 0).
    std::size_t currentGear() const      { return currentGear_; }
    void setCurrentGear(std::size_t g)   { currentGear_ = g; }

    // Shift one gear up/down, clamped to valid range. Returns true if shifted.
    bool shiftUp();
    bool shiftDown();

private:
    std::vector<double> gearRatios_;
    double finalDrive_;
    double efficiency_;
    std::size_t currentGear_;
};

#endif // TRANSMISSION_H
