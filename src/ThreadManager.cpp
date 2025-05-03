#include "../include/ThreadManager.h"
#include <stdexcept>

ThreadManager::ThreadManager(size_t numThreads) 
    : numThreads(numThreads), running(false) {
    if (numThreads <= 0) {
        throw std::invalid_argument("Number of threads must be positive");
    }
    threadLoads.resize(numThreads);
    for (auto& load : threadLoads) {
        load = 0;
    }
}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    if (running) return;
    running = true;
    threads.clear();
    threads.reserve(numThreads);
    for (size_t i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadManager::workerThread, this, i);
    }
}

void ThreadManager::stop() {
    if (!running) return;
    running = false;
    taskCondition.notify_all();
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }
    threads.clear();
}

void ThreadManager::addTask(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(taskMutex);
        taskQueue.push(std::move(task));
    }
    taskCondition.notify_one();
}

bool ThreadManager::isRunning() const {
    return running;
}

size_t ThreadManager::getNumThreads() const {
    return numThreads;
}

void ThreadManager::setNumThreads(size_t newNumThreads) {
    stop();
    numThreads = newNumThreads;
    {
        std::lock_guard<std::mutex> lock(threadLoadsMutex);
        threadLoads.resize(numThreads);
        for (auto& load : threadLoads) load = 0;
    }
    start();
}

size_t ThreadManager::getTaskCount() const {
    std::lock_guard<std::mutex> lock(taskMutex);
    return taskQueue.size();
}

void ThreadManager::waitForCompletion() {
    std::unique_lock<std::mutex> lock(completionMutex);
    taskCondition.wait(lock, [this] {
        std::lock_guard<std::mutex> tlock(taskMutex);
        return taskQueue.empty() && activeThreads == 0;
    });
}

size_t ThreadManager::getActiveThreadCount() const {
    return activeThreads;
}

void ThreadManager::workerThread(size_t threadId) {
    while (running) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(taskMutex);
            taskCondition.wait(lock, [this] { return !taskQueue.empty() || !running; });
            if (!running && taskQueue.empty()) return;
            task = std::move(taskQueue.front());
            taskQueue.pop();
            ++activeThreads;
            {
                std::lock_guard<std::mutex> loadsLock(threadLoadsMutex);
                ++threadLoads[threadId];
            }
        }
        if (task) task();
        {
            std::lock_guard<std::mutex> lock(taskMutex);
            --activeThreads;
            if (taskQueue.empty() && activeThreads == 0) {
                taskCondition.notify_all();
            }
        }
    }
}

void ThreadManager::processNextTask() {
    // Not used directly; tasks are processed by worker threads.
} 