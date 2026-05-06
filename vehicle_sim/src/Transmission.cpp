// Transmission.cpp
#include "Transmission.h"

Transmission::Transmission()
    : finalDrive_(3.31), efficiency_(0.85), currentGear_(0) {}

Transmission::Transmission(const std::vector<double>& gearRatios,
                           double finalDrive,
                           double efficiency)
    : gearRatios_(gearRatios),
      finalDrive_(finalDrive),
      efficiency_(efficiency),
      currentGear_(0) {}

double Transmission::totalRatio(std::size_t gearIdx) const {
    if (gearRatios_.empty()) return 0.0;
    if (gearIdx >= gearRatios_.size()) gearIdx = gearRatios_.size() - 1;
    return gearRatios_[gearIdx] * finalDrive_;
}

bool Transmission::shiftUp() {
    if (currentGear_ + 1 < gearRatios_.size()) {
        ++currentGear_;
        return true;
    }
    return false;
}

bool Transmission::shiftDown() {
    if (currentGear_ > 0) {
        --currentGear_;
        return true;
    }
    return false;
}
