#include "scheduler.hh"
#include "constants.hh"  // Defines PS2_CYCLES_PER_FRAME (e.g. 4915200) and other constants if needed.
#include <chrono>
#include <thread>
#include <iostream>

void Scheduler::add_task(Task task, uint64_t interval) {
    // Initialize next_execution based on the current cycle count.
    tasks.push_back({ task, interval, current_cycle + interval });
}

void Scheduler::tick() {
    auto now = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(now - last_frame_time).count();

    constexpr uint64_t PS2_CPU_FREQ = 294912000 / 1000; // cycles per second
    uint64_t cycles_to_run = static_cast<uint64_t>(elapsed_seconds * PS2_CPU_FREQ);

    if (cycles_to_run == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
    }

    current_cycle += cycles_to_run;

    for (auto& task : tasks) {
        //while (current_cycle >= task.next_execution) {
            if (current_cycle >= task.next_execution) {
                //std::cout << "Executing task scheduled for interval: " << task.interval << " cycles." << std::endl;
                task.task(cycles_to_run);
                task.next_execution += task.interval;
            }
        //}
    }

    if (current_cycle >= PS2_CYCLES_PER_FRAME) {
        auto frame_period = std::chrono::duration<double>(1.0 / 60.0);  // 60Hz
        auto target_time = last_frame_time + std::chrono::duration_cast<std::chrono::nanoseconds>(frame_period);
        now = std::chrono::steady_clock::now();
        /*if (now < target_time) {
            std::this_thread::sleep_for(target_time - now);
        }*/
        current_cycle -= PS2_CYCLES_PER_FRAME;
        for (auto& task : tasks) {
            if (task.next_execution >= PS2_CYCLES_PER_FRAME)
                task.next_execution -= PS2_CYCLES_PER_FRAME;
        }
        last_frame_time = std::chrono::steady_clock::now();
    }
}
