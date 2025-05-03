#pragma once

#include "Particle.h"
#include "ContainmentField.h"
#include "ThreadManager.h"
#include "Config.h"
#include <vector>
#include <memory>
#include <atomic>
#include <mutex>
#include <condition_variable>

class Simulation {
public:
    Simulation(const Config& config);
    ~Simulation();

    void initializeParticles(const Config& config);
    void setContainmentField(std::unique_ptr<ContainmentField> field);

    void start();
    void stop();
    void step();

    void addParticle(std::unique_ptr<Particle> particle);
    void removeEscapedParticles();
    size_t getParticleCount() const;
    const std::vector<std::unique_ptr<Particle>>& getParticles() const;

    double getTotalEnergy() const;

    void setNumThreads(size_t numThreads);
    size_t getNumThreads() const;
    const ThreadManager& getThreadManager() const { return *threadManager; }

private:
    void parallelUpdatePositions(double timeStep);
    void parallelApplyForces(double timeStep);
    void parallelHandleCollisions();

    std::vector<std::unique_ptr<Particle>> particles;
    std::unique_ptr<ContainmentField> containmentField;
    std::unique_ptr<ThreadManager> threadManager;
    double fieldSize;
    const double timeStep;
    std::atomic<bool> running{false};
    size_t numThreads;
    mutable std::mutex simulationMutex;
}; 