#pragma once

#include <cstdint>
#include <iop/iop.hh>
#include <iostream>

class IOP;

void iop_step_interpreter(IOP *iop);
void iop_interpreter_setup(IOP *iop);
