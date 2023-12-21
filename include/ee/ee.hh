#pragma once

#include <bus/bus.hh>
#include <cstdint>
#include <ee/ee_interpreter.hh>
#include <functional>

enum class EmulationMode
{
    Interpreter,
    CachedInterpreter,
    DynamicRecompiler
};

class EE
{
  private:
  public:
    EE(Bus *bus_, EmulationMode mode = EmulationMode::Interpreter);
    ~EE();

    std::function<void()> ee_step;

    void run();

    Bus *bus;
    std::uint32_t pc;
};
