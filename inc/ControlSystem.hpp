#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include "customBlocks/PD_controller.hpp"
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1,E2 Ewl, Ewr;
    PDController<> controller1, controller2;
    D<> d1, d2;
    Gain<> i1, i2, i1_inv, i2_inv, kM1, kM2, kM1_inv, kM2_inv, R1, R2;
    Saturation<> qdMax1, qdMax2, QMax1, QMax2;
    PeripheralOutput<> M1, M2, Mwl, Mwr;
    Sum<> U1, U2;
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP