#ifndef PICONTROLLER_HPP_
#define PICONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Sum.hpp>

// include header files for the inputs, outputs, and subblocks

using namespace eeros::control;

template <typename T = double>
class PIController : public Block
{
public:
    // Constructor for symmetric limits
    PIController(double f_Task, double D, double s, double M, double ILimit)
        : Kp(f_Task / s),
          KI(f_Task / 2.0 / s / D * f_Task / 2.0 / s / D),
          M(M)
    {
        init(ILimit, -ILimit);
    }
    // Constructor for asymmetric limits
    PIController(double f_Task, double D, double s, double M, double IUpperLimit, double ILowerLimit)
    : Kp(f_Task / s),
          KI(f_Task / 2.0 / s / D * f_Task / 2.0 / s / D),
          M(M)
    {
        init(ILimit, -ILimit);
    }


    // Implement getter functions for inputs and outputs
    Input<> &getInqds() { return ed.getIn(0); }
    Input<> &getInqd()  { return qd.getIn(); }

    Output<> &getOutQ()  { return M.getOut(); }
    Output<> &getOutqd() { return qd.getOut(); }

    virtual void run()
    {
        // Calculate output values, set timestamps and
        // call the run method of the subblocks
        qd.run();
        ed.run();
        Kp.run();
        e.run();
        KI.run();
        qddc.run();
        M.run();
    }

    // Enable/disable the integrator
    void enableIntegrator(void) {e.enable(); }
    void disableIntegrator(void) {e.disable(); }

    // Define inputs, outputs, intermediate variables and subblocks
    Sum<2,T> ed, qddC;
    Gain<T> qd, Kp, KI, M;
    I<T> e;

private:
    void init(double IUpperLimit, double ILowerLimit)
    {
        // Name all Blocks
        ed.setName("PIController->ed");
        qddC.setName("PIController->qddC");
        qd.setName("PIController->qd");
        Kp.setName("PIController->Kp");
        KI.setName("PIController->KI");
        e.setName("PIController->e");
        M.setName("PIController->M");
        // Name all Signals
        ed.getOut().getSignal().setName("Wheel velocity error [m/s]");
        qddC.getOut().getSignal().setName("Wheel acceleration setpoint [m/s²]");
        qd.getOut().getSignal().setName("Wheel velocity setpoint [m/s]");
        Kp.getOut().getSignal().setName("Wheel acceleration setpoint from proportional controller part [m/s]");
        KI.getOut().getSignal().setName("Wheel acceleration setpoint from integrative controller part [m/s]");
        e.getOut().getSignal().setName("Wheel position error [m]");
        M.getOut().getSignal().setName("Wheel force setpoint [N]");
         // Connect Signals
        ed.negateInput(1);
        ed.getIn(1)connect(qd.getOut());
        Kp.getIn().connect(ed.getOut());
        e.getIn().connect(ed.getOut());
        e.setInitCondition(0.0);
        e.setLimit(IUpperLimit, ILowerLimit);
        KI.getIn().connect(e.getOut());
        qddC.getIn(0).connect(KI.getOut());
        qddC.getIn(1).connect(Kp.getOut());
        this->M.getIn().connect(qddC.getOut());

    }
    
    
};

#endif //CUSTOMBLOCKTEMPLATE_HPP_
