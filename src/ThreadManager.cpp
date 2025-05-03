#include "../include/ThreadManager.h"
#include <stdexcept>
#include <iostream>

ThreadManager::ThreadManager(size_t numThreads)
    : numThreads(numThreads), running(false), unfinishedTasks(0) {}

ThreadManager::~ThreadManager() {
    stop();
}

void ThreadManager::start() {
    if (running) return;
    running = true;
    threads.clear();
    for (size_t i = 0; i < numThreads; ++i) {
        threads.emplace_back(&ThreadManager::workerThread, this);
    }
}

void ThreadManager::stop() {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        running = false;
    }
    taskAvailable.notify_all();
    for (auto& t : threads) {
        if (t.joinable()) t.join();
    }
    threads.clear();
}

void ThreadManager::addTask(std::function<void()> task) {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (!running) throw std::runtime_error("ThreadManager not running");
        taskQueue.push(std::move(task));
        ++unfinishedTasks;
    }
    taskAvailable.notify_one();
}

void ThreadManager::waitForCompletion() {
    std::unique_lock<std::mutex> lock(queueMutex);
    allTasksDone.wait(lock, [this]() { return unfinishedTasks == 0 && taskQueue.empty(); });
}

void ThreadManager::setNumThreads(size_t n) {
    stop();
    numThreads = n;
    start();
}

size_t ThreadManager::getNumThreads() const {
    return numThreads;
}

bool ThreadManager::isRunning() const {
    return running;
}

void ThreadManager::workerThread() {
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(queueMutex);
            taskAvailable.wait(lock, [this]() { return !taskQueue.empty() || !running; });
            if (!running && taskQueue.empty()) return;
            if (!taskQueue.empty()) {
                task = std::move(taskQueue.front());
                taskQueue.pop();
            }
        }
        if (task) {
            task();
            std::lock_guard<std::mutex> lock(queueMutex);
            --unfinishedTasks;
            if (unfinishedTasks == 0 && taskQueue.empty()) {
                allTasksDone.notify_all();
            }
        }
    }
}