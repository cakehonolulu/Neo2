#pragma once

#include <bus/bus.hh>
#include <cstdint>
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
    std::function<void()> ee_step;

  public:
    EE(Bus *bus_, EmulationMode mode = EmulationMode::Interpreter);
    ~EE();

    void run();

    Bus *bus;
    std::uint32_t pc;

    void ee_step_interpreter();
    std::uint32_t fetch_ee_opcode();
    void parse_ee_opcode(std::uint32_t opcode);
};
