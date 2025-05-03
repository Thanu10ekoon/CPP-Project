#include "../include/Simulation.h"
#include "../include/Config.h"
#include <algorithm>
#include <random>
#include <iostream>
#include <future>
#include <cmath>

Simulation::Simulation(const Config& config)
    : fieldSize(config.field_size),
      timeStep(config.time_step),
      containmentField(std::make_unique<ContainmentField>(config)),
      threadManager(std::make_unique<ThreadManager>(config.initial_threads)),
      numThreads(config.initial_threads) {
    initializeParticles(config);
    threadManager->start();
}

Simulation::~Simulation() {
    stop();
}

void Simulation::initializeParticles(const Config& config) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-fieldSize/4, fieldSize/4); // Start closer to center
    std::uniform_real_distribution<> vel_dis(-0.5, 0.5); // Lower initial velocities
    size_t count = config.num_particles;
    for (size_t i = 0; i < count; ++i) {
        auto particle = std::make_unique<Particle>(
            dis(gen), dis(gen),
            config.initial_energy,
            config.particle_radius,
            config.max_energy
        );
        particle->setVelocity(vel_dis(gen), vel_dis(gen));
        particles.push_back(std::move(particle));
    }
    std::cout << "Initialized " << particles.size() << " particles." << std::endl;
}

void Simulation::setContainmentField(std::unique_ptr<ContainmentField> field) {
    containmentField = std::move(field);
}

void Simulation::start() {
    running = true;
    threadManager->start();
    std::cout << "Simulation started with " << numThreads << " threads." << std::endl;
}

void Simulation::stop() {
    running = false;
    threadManager->stop();
    std::cout << "Simulation stopped." << std::endl;
}

void Simulation::step() {
    removeEscapedParticles();
    parallelApplyForces(timeStep);
    parallelUpdatePositions(timeStep);
    parallelHandleCollisions();
    threadManager->waitForCompletion();
}

void Simulation::addParticle(std::unique_ptr<Particle> particle) {
    std::lock_guard<std::mutex> lock(simulationMutex);
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles() {
    std::lock_guard<std::mutex> lock(simulationMutex);
    particles.erase(std::remove_if(particles.begin(), particles.end(),
        [this](const std::unique_ptr<Particle>& p) {
            return !containmentField->isParticleContained(*p);
        }), particles.end());
}

size_t Simulation::getParticleCount() const {
    return particles.size();
}

const std::vector<std::unique_ptr<Particle>>& Simulation::getParticles() const {
    return particles;
}

double Simulation::getTotalEnergy() const {
    double total = 0.0;
    for (const auto& particle : particles) {
        total += particle->getEnergy();
    }
    return total;
}

void Simulation::setNumThreads(size_t newNumThreads) {
    numThreads = newNumThreads;
    threadManager->setNumThreads(newNumThreads);
}

size_t Simulation::getNumThreads() const {
    return numThreads;
}

// --- Parallelized simulation steps using ThreadManager ---
void Simulation::parallelUpdatePositions(double dt) {
    size_t n = particles.size();
    size_t chunk = (n + numThreads - 1) / numThreads;
    for (size_t t = 0; t < numThreads; ++t) {
        size_t start = t * chunk;
        size_t end = std::min(start + chunk, n);
        if (start >= end) continue;
        threadManager->addTask([this, start, end, dt]() {
            for (size_t i = start; i < end; ++i) {
                auto& particle = particles[i];
                double x = particle->getX() + particle->getVX() * dt;
                double y = particle->getY() + particle->getVY() * dt;
                particle->setPosition(x, y);
            }
        });
    }
}

void Simulation::parallelApplyForces(double dt) {
    size_t n = particles.size();
    size_t chunk = (n + numThreads - 1) / numThreads;
    for (size_t t = 0; t < numThreads; ++t) {
        size_t start = t * chunk;
        size_t end = std::min(start + chunk, n);
        if (start >= end) continue;
        threadManager->addTask([this, start, end, dt]() {
            for (size_t i = start; i < end; ++i) {
                auto& particle = particles[i];
                double x = particle->getX();
                double y = particle->getY();
                
                // Apply containment force
                double forceMag = containmentField->getContainmentForce(*particle);
                double r = std::sqrt(x*x + y*y);
                if (r > 0.0) {
                    double fx = -forceMag * (x / r);
                    double fy = -forceMag * (y / r);
                    
                    // Update velocity with force
                    double vx = particle->getVX() + fx * dt;
                    double vy = particle->getVY() + fy * dt;
                    
                    // Apply damping to prevent excessive speeds
                    double speed = std::sqrt(vx*vx + vy*vy);
                    double maxSpeed = 2.0;
                    if (speed > maxSpeed) {
                        vx *= maxSpeed / speed;
                        vy *= maxSpeed / speed;
                    }
                    
                    particle->setVelocity(vx, vy);
                }
            }
        });
    }
}

void Simulation::parallelHandleCollisions() {
    size_t n = particles.size();
    size_t chunk = (n + numThreads - 1) / numThreads;
    double radius = (particles.empty() ? 1.0 : particles[0]->getRadius());
    
    for (size_t t = 0; t < numThreads; ++t) {
        size_t start = t * chunk;
        size_t end = std::min(start + chunk, n);
        if (start >= end) continue;
        
        threadManager->addTask([this, start, end, radius]() {
            for (size_t i = start; i < end; ++i) {
                for (size_t j = i + 1; j < particles.size(); ++j) {
                    double dx = particles[i]->getX() - particles[j]->getX();
                    double dy = particles[i]->getY() - particles[j]->getY();
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    if (dist < 2.0 * radius) {
                        // Calculate collision normal
                        double nx = dx / dist;
                        double ny = dy / dist;
                        
                        // Calculate relative velocity
                        double dvx = particles[i]->getVX() - particles[j]->getVX();
                        double dvy = particles[i]->getVY() - particles[j]->getVY();
                        
                        // Calculate impulse
                        double impulse = 2.0 * (dvx * nx + dvy * ny) / 2.0;
                        
                        // Update velocities
                        double vxi = particles[i]->getVX() - impulse * nx;
                        double vyi = particles[i]->getVY() - impulse * ny;
                        double vxj = particles[j]->getVX() + impulse * nx;
                        double vyj = particles[j]->getVY() + impulse * ny;
                        
                        particles[i]->setVelocity(vxi, vyi);
                        particles[j]->setVelocity(vxj, vyj);
                        
                        // Separate particles to prevent sticking
                        double overlap = 2.0 * radius - dist;
                        double moveX = overlap * nx * 0.5;
                        double moveY = overlap * ny * 0.5;
                        
                        particles[i]->setPosition(
                            particles[i]->getX() + moveX,
                            particles[i]->getY() + moveY
                        );
                        particles[j]->setPosition(
                            particles[j]->getX() - moveX,
                            particles[j]->getY() - moveY
                        );
                    }
                }
            }
        });
    }
} 