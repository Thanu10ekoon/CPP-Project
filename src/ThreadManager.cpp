#include "../include/ThreadManager.h"
#include <stdexcept>
#include <iostream>

ThreadManager::ThreadManager(size_t numThreads) 
    : numThreads(numThreads), running(false) {
    if (numThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    
    // Initialize thread load counters
    threadLoads.resize(numThreads);
    for (auto& load : threadLoads) {
        load = 0;
    }
}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    std::lock_guard<std::mutex> lock(taskMutex);
    if (running) {
        return; // Already started
    }
    
    running = true;
    threads.clear(); // Clear any existing threads
    
    // Create worker threads
    for (size_t i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadManager::workerThread, this, i);
    }
}

void ThreadManager::stop() {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (!running) {
            return; // Already stopped
        }
        running = false;
    }
    
    // Notify all waiting threads to check running flag
    taskCondition.notify_all();
    
    // Join all threads
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    threads.clear();
    
    // Clear any remaining tasks
    std::lock_guard<std::mutex> lock(taskMutex);
    while (!taskQueue.empty()) {
        taskQueue.pop();
    }
}

void ThreadManager::addTask(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (!running) {
            throw std::runtime_error("Cannot add task when ThreadManager is not running");
        }
        taskQueue.push(std::move(task));
    }
    
    // Notify one waiting thread that a new task is available
    taskCondition.notify_one();
}

bool ThreadManager::isRunning() const {
    return running;
}

size_t ThreadManager::getNumThreads() const {
    return numThreads;
}

void ThreadManager::setNumThreads(size_t newNumThreads) {
    if (newNumThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    
    if (running) {
        stop();
        numThreads = newNumThreads;
        threadLoads.resize(numThreads);
        for (auto& load : threadLoads) {
            load = 0;
        }
        start();
    } else {
        numThreads = newNumThreads;
        threadLoads.resize(numThreads);
        for (auto& load : threadLoads) {
            load = 0;
        }
    }
}

size_t ThreadManager::getTaskCount() const {
    std::lock_guard<std::mutex> lock(taskMutex);
    return taskQueue.size();
}

void ThreadManager::waitForCompletion() {
    std::unique_lock<std::mutex> lock(completionMutex);
    while (running && activeThreads > 0) {
        taskCondition.wait(lock);
    }
}

size_t ThreadManager::getActiveThreadCount() const {
    return activeThreads;
}

void ThreadManager::workerThread(size_t threadId) {
    while (running) {
        std::function<void()> task;
        
        // Wait for and get a task
        {
            std::unique_lock<std::mutex> lock(taskMutex);
            while (running && taskQueue.empty()) {
                taskCondition.wait(lock);
            }
            
            if (!running) {
                break; // ThreadManager is shutting down
            }
            
            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
                activeThreads++;
                threadLoads[threadId]++;
            }
        }
        
        // Execute the task if we got one
        if (task) {
            try {
                task();
            } catch (const std::exception& e) {
                std::cerr << "Error in thread " << threadId << ": " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "Unknown error in thread " << threadId << std::endl;
            }
            
            // Decrement active threads counter
            activeThreads--;
            
            // Notify any waiting threads that task is complete
            taskCondition.notify_all();
        }
    }
}

void ThreadManager::processNextTask() {
    std::function<void()> task;
    
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        if (taskQueue.empty()) {
            return;
        }
        
        task = std::move(taskQueue.front());
        taskQueue.pop();
    }
    
    if (task) {
        activeThreads++;
        try {
            task();
        } catch (const std::exception& e) {
            std::cerr << "Error in processNextTask: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown error in processNextTask" << std::endl;
        }
        activeThreads--;
    }
}