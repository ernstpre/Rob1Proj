#ifndef InvMotMod_HPP_
#define InvMotMod_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Sum.hpp>

// include header files for the inputs, outputs, and subblocks

using namespace eeros::control;

template <typename T = double>
class InvMotMod : public Block
{
public:
    InvMotMod(double QMax, double qdMax, double i, double kM, double R)
        : QMax(QMax),
          qdMax(qdMax),
          i_inv(1.0 / i),
          kM_inv(1.0 / kM),
          R(R),
          i(i),
          kM(kM) 
    {
        // Name Block
        this->QMax.setName("InvMotMod->QMax");
        this->qdMax.setName("InvMotMod->qdMax");
        this->i_inv.setName("InvMotMod->i_inv");
        this->kM_inv.setName("InvMotMod->kM_inv");
        this->R.setName("InvMotMod->R");
        this->i.setName("InvMotMod->i");
        this->kM.setName("InvMotMod->kM");
        this->U.setName("InvMotMod->U");

        // Name Signals
        this->QMax.getOut().getSignal().setName("Saturated wheel force setpoint [N]");
        this->qdMax.getOut().getSignal().setName("Saturated wheel velocity setpoint [m/s]");
        this->i_inv.getOut().getSignal().setName("Motor torque setpoint [Nm]");
        this->kM_inv.getOut().getSignal().setName("Motor current setpoint [A]");
        this->R.getOut().getSignal().setName("Motor voltage setpoint from force [V]");
        this->i.getOut().getSignal().setName("Motor angular velocity setpoint [rad/s]");
        this->kM.getOut().getSignal().setName("Motor voltage setpoint from velocity [V]");
        this->U.getOut().getSignal().setName("Motor voltage setpoint [V]");

        // Connect subblocks, initilize variables
        i_inv.getIn().connect(QMax.getOut());
        kM_inv.getIn().connect(i_inv.getOut());
        R.getIn().connect(kM_inv.getOut());
        i.getIn().connect(qdMax.getOut());
        kM.getIn().connect(i.getOut());
        U.getIn(0).connect(R.getOut());
        U.getIn(1).connect(kM.getOut());
    }

 // Implement getter functions for inputs and outputs
    Input<> &getInQ() {return QMax.getIn(); }
    Input<> &getInqd() {return qdMax.getIn(); }

    Output<> &getOutQ() {return U.getOut(); }

   

    virtual void run()
    {
        // Calculate output values, set timestamps and
        // call the run method of the subblocks
        QMax.run();
        i_inv.run();
        kM_inv.run();
        R.run();
        qdMax.run();
        i.run();
        kM.run();
        U.run();
    }


    // Define inputs, outputs, intermediate variables and subblocks
    Saturation<T> QMax, qdMax;
    Gain<T> i_inv, kM_inv, R, i, kM;
    Sum<2,T> U;
};

#endif //CUSTOMBLOCKTEMPLATE_HPP_
