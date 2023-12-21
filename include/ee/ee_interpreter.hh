#pragma once

#include <cstdint>
#include <ee/ee.hh>
#include <iostream>

class EE;

void ee_step_interpreter(EE *ee);
std::uint32_t inline fetch_ee_opcode(EE *ee);
void parse_ee_opcode(EE *ee, std::uint32_t opcode);
