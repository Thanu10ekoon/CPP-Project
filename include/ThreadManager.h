#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

class ThreadManager {
public:
    explicit ThreadManager(size_t numThreads);
    ~ThreadManager();

    void start();
    void stop();
    void addTask(std::function<void()> task);
    void waitForCompletion();
    void setNumThreads(size_t numThreads);
    size_t getNumThreads() const;
    bool isRunning() const;

private:
    void workerThread();
    std::vector<std::thread> threads;
    std::queue<std::function<void()>> taskQueue;
    std::mutex queueMutex;
    std::condition_variable taskAvailable;
    std::condition_variable allTasksDone;
    bool running = false;
    size_t numThreads;
    size_t unfinishedTasks = 0;
};