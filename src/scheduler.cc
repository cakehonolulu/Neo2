#include "scheduler.hh"
#include "constants.hh"
#include <algorithm>
#include <limits>
#include <chrono>
#include <thread>
#include <iostream>
#include <log/log.hh>

using TimestampLimit = std::numeric_limits<int64_t>;

Scheduler::Scheduler() {
    reset();
}

void Scheduler::reset() {
    registered_funcs.clear();
    ee_cycles.count = 0;
    bus_cycles.count = 0;
    iop_cycles.count = 0;

    ee_cycles.remainder = 0;
    bus_cycles.remainder = 0;
    iop_cycles.remainder = 0;

    next_event_id = 0;
    closest_event_time = TimestampLimit::max();

    events.clear();
    timers.clear();

    timer_event_id = register_function([this](uint64_t param) { timer_event(param); });
}

unsigned int Scheduler::calculate_run_cycles() {
    if (events.empty())
        throw std::runtime_error("[Scheduler] No events registered");

    const static int MAX_CYCLES = 32;
    if (ee_cycles.count + MAX_CYCLES <= closest_event_time)
        run_cycles = MAX_CYCLES;
    else {
        int64_t delta = closest_event_time - ee_cycles.count;
        run_cycles = delta > 0 ? delta : 0;
    }

    return run_cycles;
}

unsigned int Scheduler::get_bus_run_cycles() {
    unsigned int bus_run_cycles = run_cycles >> 1;
    if (bus_cycles.remainder && (run_cycles & 0x1))
        bus_run_cycles++;

    return bus_run_cycles;
}

unsigned int Scheduler::get_iop_run_cycles() {
    unsigned int iop_run_cycles = run_cycles >> 3;
    if (iop_cycles.remainder + (run_cycles & 0x7) >= 8)
        iop_run_cycles++;

    return iop_run_cycles;
}

int Scheduler::register_function(std::function<void(uint64_t)> func) {
    int id = registered_funcs.size();
    registered_funcs.push_back(func);
    return id;
}

int Scheduler::register_timer_callback(std::function<void(uint64_t, bool)> cb) {
    int id = timer_callbacks.size();
    timer_callbacks.push_back(cb);
    return id;
}

uint64_t Scheduler::add_event(int func_id, uint64_t delta, const std::string& identifier, uint64_t param) {
    if (func_id < 0 || func_id >= registered_funcs.size())
        throw std::runtime_error("[Scheduler] Out-of-bounds func_id given in add_event");

    SchedulerEvent event;
    event.func_id = func_id;
    event.time_to_run = ee_cycles.count + delta;
    event.param = param;
    event.event_id = next_event_id++;
    event.identifier = identifier; // Set the identifier

    closest_event_time = std::min(event.time_to_run, closest_event_time);
    events.push_back(event);

    return event.event_id;
}

void Scheduler::delete_event(uint64_t event_id) {
    auto it = std::find_if(events.begin(), events.end(), [event_id](const SchedulerEvent& event) {
        return event.event_id == event_id;
    });

    if (it != events.end())
        events.erase(it);
    else
        throw std::runtime_error("[Scheduler] No event ID found in delete_event");
}

uint64_t Scheduler::convert_to_ee_cycles(uint64_t cycles, uint64_t clockrate) {
    return cycles * EE_CLOCKRATE / clockrate;
}

int64_t Scheduler::calculate_timer_event_delta(uint64_t timer_id) {
    if (timers[timer_id].paused)
        return TimestampLimit::max() >> 1;

    uint64_t overflow_mask = timers[timer_id].overflow_mask;
    int64_t overflow_delta = convert_to_ee_cycles((overflow_mask + 1) - timers[timer_id].counter, timers[timer_id].clockrate) - (timers[timer_id].remainder_clocks >> 8);

    if (timers[timer_id].target <= timers[timer_id].counter)
        return std::max(overflow_delta, (int64_t)8LL);

    int64_t target_delta = convert_to_ee_cycles(std::abs(timers[timer_id].target - timers[timer_id].counter) & overflow_mask, timers[timer_id].clockrate) - (timers[timer_id].remainder_clocks >> 8);

    return std::max(std::min(overflow_delta, target_delta), (int64_t)8LL);
}

void Scheduler::update_timer_event_time(uint64_t timer_id) {
    int64_t time = ee_cycles.count + calculate_timer_event_delta(timer_id);
    SchedulerEvent* event = get_event_ptr(timers[timer_id].event_id);
    event->time_to_run = time;
    closest_event_time = std::min(time, closest_event_time);
}

void Scheduler::update_timer_counter(uint64_t timer_id) {
    if (timers[timer_id].paused)
        return;

    int64_t delta = (ee_cycles.count - timers[timer_id].last_update) << 8;
    timers[timer_id].last_update = ee_cycles.count;

    uint64_t clock_scale = (EE_CLOCKRATE << 8) / timers[timer_id].clockrate;
    timers[timer_id].remainder_clocks += delta % clock_scale;
    int64_t remainder_delta = timers[timer_id].remainder_clocks / clock_scale;
    delta /= clock_scale;
    if (remainder_delta > 0) {
        timers[timer_id].remainder_clocks %= clock_scale;
        delta += remainder_delta;
    }

    timers[timer_id].counter += delta;
}

void Scheduler::timer_event(uint64_t index) {
    uint32_t old_counter = timers[index].counter;
    timers[index].counter = get_timer_counter(index);

    int cb_id = timers[index].callback_id;

    if ((old_counter <= timers[index].target && timers[index].counter >= timers[index].target) ||
        (old_counter >= timers[index].target && timers[index].counter >= (timers[index].target | (timers[index].overflow_mask + 1)))) {
        if (timers[index].can_target)
            timer_callbacks[cb_id](timers[index].param, false);
    }

    if (timers[index].counter > timers[index].overflow_mask) {
        timers[index].counter -= timers[index].overflow_mask + 1;
        if (timers[index].can_overflow)
            timer_callbacks[cb_id](timers[index].param, true);
    }

    restart_timer(index);
}

Scheduler::SchedulerEvent* Scheduler::get_event_ptr(uint64_t event_id) {
    auto it = std::find_if(events.begin(), events.end(), [event_id](const SchedulerEvent& event) {
        return event.event_id == event_id;
    });

    if (it != events.end())
        return &(*it);
    else
        throw std::runtime_error("[Scheduler] No event ID found in get_event_ptr");
}

uint64_t Scheduler::create_timer(int callback_id, uint64_t overflow_mask, uint64_t param) {
    if (callback_id < 0 || callback_id >= timer_callbacks.size())
        throw std::runtime_error("[Scheduler] Out-of-bounds callback ID given in create_timer");

    SchedulerTimer timer;
    timer.counter = 0;
    timer.target = TimestampLimit::max();
    timer.clockrate = 1;
    timer.paused = true;
    timer.last_update = ee_cycles.count;
    timer.overflow_mask = overflow_mask;
    timer.remainder_clocks = 0;
    timer.event_id = add_event(timer_event_id, TimestampLimit::max(), "Timer Event", timers.size());
    timer.param = param;
    timer.callback_id = callback_id;
    timer.can_overflow = false;
    timer.can_target = false;

    timers.push_back(timer);
    return timers.size() - 1;
}

void Scheduler::restart_timer(uint64_t timer_id) {
    SchedulerTimer* t = &timers[timer_id];
    t->event_id = add_event(timer_event_id, TimestampLimit::max(), "Timer Event", timer_id);
    update_timer_event_time(timer_id);
}

uint64_t Scheduler::get_timer_counter(uint64_t timer_id) {
    if (!timers[timer_id].paused)
        update_timer_counter(timer_id);

    return timers[timer_id].counter;
}

void Scheduler::set_timer_pause(uint64_t timer_id, bool paused) {
    if (paused == timers[timer_id].paused)
        return;

    if (paused) {
        update_timer_counter(timer_id);
        SchedulerEvent* event = get_event_ptr(timers[timer_id].event_id);
        event->time_to_run = TimestampLimit::max();
    } else {
        update_timer_event_time(timer_id);
        timers[timer_id].last_update = ee_cycles.count;
    }

    timers[timer_id].paused = paused;
}

void Scheduler::set_timer_int_mask(uint64_t timer_id, bool can_overflow, bool can_target) {
    timers[timer_id].can_overflow = can_overflow;
    timers[timer_id].can_target = can_target;
}

void Scheduler::set_timer_counter(uint64_t timer_id, uint64_t counter) {
    if (!timers[timer_id].paused)
        update_timer_counter(timer_id);
    timers[timer_id].counter = counter;
    if (!timers[timer_id].paused)
        update_timer_event_time(timer_id);
}

void Scheduler::set_timer_target(uint64_t timer_id, uint64_t target) {
    if (!timers[timer_id].paused)
        update_timer_counter(timer_id);
    timers[timer_id].target = target;
    if (!timers[timer_id].paused)
        update_timer_event_time(timer_id);
}

void Scheduler::set_timer_clockrate(uint64_t timer_id, uint64_t clockrate) {
    if (!timers[timer_id].paused)
        update_timer_counter(timer_id);
    timers[timer_id].clockrate = clockrate;
    timers[timer_id].remainder_clocks = 0;
    if (!timers[timer_id].paused)
        update_timer_event_time(timer_id);
}

void Scheduler::update_cycle_counts() {
    ee_cycles.count += run_cycles;
    bus_cycles.count += run_cycles >> 1;
    iop_cycles.count += run_cycles >> 3;

    bus_cycles.remainder += run_cycles & 0x1;
    if (bus_cycles.remainder > 1) {
        bus_cycles.count++;
        bus_cycles.remainder = 0;
    }

    iop_cycles.remainder += run_cycles & 0x7;
    if (iop_cycles.remainder >= 8) {
        iop_cycles.count++;
        iop_cycles.remainder -= 8;
    }
}

void Scheduler::process_events() {
    fflush(stdout);
    if (ee_cycles.count >= closest_event_time) {
        int64_t new_time = 0x7FFFFFFFULL << 32ULL;
        for (auto it = events.begin(); it != events.end(); ) {
            if (it->time_to_run <= closest_event_time) {
                std::function<void(uint64_t)> func = registered_funcs[it->func_id];
                //Logger::info("Processing event: " + it->identifier + " (func_id=" + std::to_string(it->func_id) + ", param=" + std::to_string(it->param) + ")");
                func(it->param);
                it = events.erase(it);
            } else {
                new_time = std::min(it->time_to_run, new_time);
                it++;
            }
        }
        closest_event_time = new_time;
    }
}
