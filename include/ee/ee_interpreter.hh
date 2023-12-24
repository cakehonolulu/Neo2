#pragma once

#include <cstdint>
#include <ee/ee.hh>
#include <iostream>

class EE;

void ee_step_interpreter(EE *ee);
void ee_interpreter_setup(EE *ee);

void ee_interp_mfc0(EE *ee, std::uint32_t opcode);
