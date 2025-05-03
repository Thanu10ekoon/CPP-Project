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
    
}

void ContainmentField::initializeField() {
    fieldData.resize(GRID_SIZE * GRID_SIZE, 0.0); 
}

double ContainmentField::getContainmentForce(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    double distance = std::sqrt(x*x + y*y);
    double maxRadius = size / 2.0;
    
    if (distance < 1e-10) {
        return 0.0; // No force at center
    }
    
    // Strong repulsive force near the boundary
    double boundaryDistance = maxRadius - distance;
    if (boundaryDistance < 0) {
        return fieldStrength * 10.0; // Strong force to push particles back in
    }
    
    // Gentle guiding force towards center
    return fieldStrength * (1.0 - distance/maxRadius) * 0.5;
}

bool ContainmentField::isParticleContained(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    double distanceFromCenter = std::sqrt(x*x + y*y);
    double maxRadius = size / 2.0;
    
    return distanceFromCenter < maxRadius * 1.1; // Allow slight overshoot
}

void ContainmentField::update(double dt) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    for (size_t i = 0; i < fieldData.size(); ++i) {
        fieldData[i] *= (1.0 - decayRate * dt);
    }
}

void ContainmentField::setFieldStrength(double strength) {
    fieldStrength = strength;
}

double ContainmentField::getFieldStrength() const {
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