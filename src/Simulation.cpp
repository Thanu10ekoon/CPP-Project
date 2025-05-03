#include "../include/Simulation.h"
#include "../include/Config.h"
#include <algorithm>
#include <random>
#include <thread>
#include <iostream>
#include <cmath>

Simulation::Simulation(const Config& config)
    : fieldSize(config.field_size),
      timeStep(config.time_step),
      containmentField(std::make_unique<ContainmentField>(config)),
      threadManager(std::make_unique<ThreadManager>(config.initial_threads)),
      numThreads(config.initial_threads) {
    initializeParticles(config);
}

Simulation::~Simulation() {
    stop();
}

void Simulation::initializeParticles(const Config& config) {
    std::random_device rd;
    std::mt19937 gen(config.random_seed ? config.random_seed : rd());
    std::uniform_real_distribution<> pos_dis(-fieldSize/4, fieldSize/4); // Starting closer to center
    std::uniform_real_distribution<> vel_dis(-1.0, 1.0); // Velocity range
    
    for (size_t i = 0; i < config.num_particles; ++i) {
        auto particle = std::make_unique<Particle>(
            pos_dis(gen), pos_dis(gen),
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
    // Process the simulation step using our ThreadManager
    if (numThreads > 1) {
        // Parallel processing using ThreadManager
        auto particleCount = particles.size();
        auto particlesPerThread = (particleCount + numThreads - 1) / numThreads;
        
        // Process particles in chunks
        for (size_t startIdx = 0; startIdx < particleCount; startIdx += particlesPerThread) {
            size_t endIdx = std::min(startIdx + particlesPerThread, particleCount);
            
            threadManager->addTask([this, startIdx, endIdx]() {
                this->processParticleRange(startIdx, endIdx);
            });
        }
        
        // Wait for all tasks to complete
        threadManager->waitForCompletion();
    } else {
        // Sequential processing
        processParticleRange(0, particles.size());
    }
    
    // Check for collisions and escaped particles
    handleCollisions();
    removeEscapedParticles();
}

void Simulation::processParticleRange(size_t startIdx, size_t endIdx) {
    // For each particle in our range
    for (size_t i = startIdx; i < endIdx; ++i) {
        if (i >= particles.size()) {
            break;
        }
        
        auto& particle = particles[i];
        
        // Apply containment forces
        applyForcesToParticle(*particle, timeStep);
        
        // Update position
        updateParticlePosition(*particle, timeStep);
    }
}

void Simulation::applyForcesToParticle(Particle& particle, double dt) {
    // Get particle position
    double x = particle.getX();
    double y = particle.getY();
    
    // Calculate distance from center
    double distance = std::sqrt(x*x + y*y);
    
    // Calculate normalized direction outward from center (away from center)
    double dirX = 0, dirY = 0;
    if (distance > 1e-10) {
        dirX = x / distance;  // Direction is away from center
        dirY = y / distance;
    }
    
    // Get force magnitude from containment field
    double forceMagnitude = containmentField->getContainmentForce(particle);
    
    // Calculate force components (pushing outward)
    double fx = dirX * forceMagnitude;
    double fy = dirY * forceMagnitude;
    
    // Update velocity based on forces
    double vx = particle.getVX() + fx * dt;
    double vy = particle.getVY() + fy * dt;
    
    particle.setVelocity(vx, vy);
}

void Simulation::updateParticlePosition(Particle& particle, double dt) {
    // Update position based on velocity
    double x = particle.getX() + particle.getVX() * dt;
    double y = particle.getY() + particle.getVY() * dt;
    
    particle.setPosition(x, y);
}

void Simulation::addParticle(std::unique_ptr<Particle> particle) {
    std::lock_guard<std::mutex> lock(particleMutex);
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles() {
    std::lock_guard<std::mutex> lock(particleMutex);
    particles.erase(
        std::remove_if(particles.begin(), particles.end(),
            [this](const std::unique_ptr<Particle>& p) {
                return !containmentField->isParticleContained(*p);
            }),
        particles.end()
    );
}

size_t Simulation::getParticleCount() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return particles.size();
}

const std::vector<std::unique_ptr<Particle>>& Simulation::getParticles() const {
    return particles;
}

double Simulation::getTotalEnergy() const {
    std::lock_guard<std::mutex> lock(particleMutex);
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

void Simulation::handleCollisions() {
    std::lock_guard<std::mutex> lock(particleMutex);
    
    // Check each particle against others for collision
    for (size_t i = 0; i < particles.size(); i++) {
        for (size_t j = i + 1; j < particles.size(); j++) {
            if (particles[i]->isColliding(*particles[j])) {
                particles[i]->collide(*particles[j]);
            }
        }
    }
}

void Simulation::workerThread(size_t threadId) {
    // This function is no longer needed as we're using the ThreadManager
    // for task distribution, but we'll keep it for compatibility
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Simulation::updatePositions(double dt) {
    // This method is now replaced by processParticleRange and updateParticlePosition
    // Keeping for backwards compatibility
    for (auto& particle : particles) {
        updateParticlePosition(*particle, dt);
    }
}

void Simulation::applyForces(double dt) {
    // This method is now replaced by processParticleRange and applyForcesToParticle
    // Keeping for backwards compatibility
    for (auto& particle : particles) {
        applyForcesToParticle(*particle, dt);
    }
}