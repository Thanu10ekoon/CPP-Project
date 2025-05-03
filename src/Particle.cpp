#include "../include/Particle.h"
#include <cmath>
#include <thread>
#include <chrono>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
    // Initialize with the correct energy value
    this->energy = energy;
}

Particle::~Particle() {
}

double Particle::getX() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return x;
}

double Particle::getY() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return y;
}

void Particle::setPosition(double newX, double newY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    x = newX;
    y = newY;
}

double Particle::getVX() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return vx;
}

double Particle::getVY() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return vy;
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return energy;
}

double Particle::getMaxEnergy() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return MAX_ENERGY;
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(newEnergy, MAX_ENERGY);
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY);
}

void Particle::collide(Particle& other) {
    std::lock_guard<std::mutex> lock(particleMutex);
    
    // Simple elastic collision - swap velocities
    double temp_vx = vx;
    double temp_vy = vy;
    
    {
        std::lock_guard<std::mutex> other_lock(other.particleMutex);
        vx = other.vx;
        vy = other.vy;
        
        other.vx = temp_vx;
        other.vy = temp_vy;
    }
}

bool Particle::isColliding(const Particle& other) const {
    std::lock_guard<std::mutex> lock(particleMutex);
    std::lock_guard<std::mutex> other_lock(other.particleMutex);
    
    double dx = x - other.x;
    double dy = y - other.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance < (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
}
