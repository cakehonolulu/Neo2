#pragma once
#include <functional>
#include <vector>
#include <cstdint>
#include <chrono>

class Scheduler {
public:
    using Task = std::function<void(uint64_t)>;

    void add_task(Task task, uint64_t interval);
    // Tick computes elapsed cycles and runs tasks accordingly.
    void tick();

private:
    struct ScheduledTask {
        Task task;
        uint64_t interval;       // How many cycles between executions.
        uint64_t next_execution; // When to next run, in cycles.
    };

    std::vector<ScheduledTask> tasks;
    // current_cycle accumulates cycles (and is never reset to zero; we subtract a frame's worth each frame).
    uint64_t current_cycle = 0;
    std::chrono::steady_clock::time_point last_frame_time = std::chrono::steady_clock::now();
};
