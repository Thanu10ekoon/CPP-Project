#include "../include/ContainmentField.h"
#include "../include/Particle.h"
#include "../include/Config.h"
#include <cmath>
#include <algorithm>

ContainmentField::ContainmentField(const Config& config)
    : size(config.field_size), fieldStrength(config.initial_strength), decayRate(config.initial_decay_rate), GRID_SIZE(config.field_grid_size), fieldEnergy(0.0) {
    initializeField();
}

ContainmentField::~ContainmentField() {
    // Clean up any energy pulses
    for (auto* pulse : energyPulses) {
        delete pulse;
    }
    energyPulses.clear();
}

void ContainmentField::initializeField() {
    fieldData.resize(GRID_SIZE * GRID_SIZE, 0.0); 
}

double ContainmentField::getContainmentForce(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Calculate distance from center
    double distance = std::sqrt(x*x + y*y);
    
    // As particles get closer to the center, the force increases
    // This will push particles away from the center
    double normalizedDistance = distance / (size/2);
    
    // Force increases as particles approach the center
    // For proper outward push, force should be directed outward from center
    if (normalizedDistance > 0.95) {
        return 0.0; // Very little force near the boundary
    }
    
    // Inverse relationship - more force near center
    return fieldStrength * (1.0 - normalizedDistance);
}

bool ContainmentField::isParticleContained(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Check if particle is within field boundaries
    return std::abs(x) <= size/2 && std::abs(y) <= size/2;
}

void ContainmentField::update(double dt) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    
    // Update field energy levels
    for (size_t i = 0; i < fieldData.size(); ++i) {
        fieldData[i] *= (1.0 - decayRate * dt);
    }
    
    // Update and remove expired energy pulses
    auto it = energyPulses.begin();
    while (it != energyPulses.end()) {
        (*it)->lifetime -= dt;
        if ((*it)->lifetime <= 0.0) {
            delete *it;
            it = energyPulses.erase(it);
        } else {
            ++it;
        }
    }
}

void ContainmentField::setFieldStrength(double strength) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldStrength = strength;
}

double ContainmentField::getFieldStrength() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldStrength;
}

void ContainmentField::setDecayRate(double rate) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    decayRate = rate;
}

double ContainmentField::getDecayRate() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return decayRate;
}

double ContainmentField::getSize() const {
    return size;
}

double ContainmentField::getFieldEnergy() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldEnergy;
}