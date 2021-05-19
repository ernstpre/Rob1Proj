#ifndef PDCONTROLLER_HPP_
#define PDCONTROLLER_HPP_
// include header files for the inputs, outputs, and subblocks
#include <eeros/control/Block.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/D.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Output.hpp>


using namespace eeros::control;

template <typename T = double>
class PDController : public Block
{
public:
    PDController(double Kp, double Kd, double M)
        : Kp(Kp), Kd(Kd), M(M) 
    {
        init();
    }

    PDController(double f_task, double D, double s, double M)
        : Kp(f_task / s / D * f_task / s / D), Kd(2.0 * f_task / s), M(M)
        {
            init();
        }
    // Implement getter functions for inputs and outputs
    
    Input<T> &getIn(uint8_t index)
    {
        if (index >= 2)
            throw eeros::Fault("index out of bounds in block '" + this-> getName() + "'");
        return in[index].getIn();
        ;
    }

    Output<T> &getOut(uint8_t index)
    {
        if (index == 0)
            return M.getOut();
        else if (index == 1)
            return d2.getOut();
        else
            throw eeros::Fault("index out of bounds in block '" + this->getName() + "'");
    }
        

    void run()
    {
        // Calculate output values, set timestamps and 
        // call the run method of the subblocks
        in[0].run();
        in[1].run();
        sum1.run();
        sum2.run();
        Kp.run();
        Kd.run();
        d1.run();
        d2.run();
        M.run();
        
    }

protected:
    // Define inputs, outputs, intermediate variables and subblocks
    Sum<2, T> sum1, sum2;
    Gain<T> Kp, Kd, M, in[2];
    D<T> d1, d2;

private:
    void init(void)
    {
        //Name all blocks
        sum1.setName("sum1");
        sum2.setName("sum2");
        Kp.setName("Kp");
        Kd.setName("Kd");
        M.setName("M");
        in[0].setName("in0");
        in[1].setName("in1");
        d1.setName("d1");
        d2.setName("d2");

        // Name all signals
        sum1.getOut().getSignal().setName("Control error [rad]");
        sum2.getOut().getSignal().setName("Acceleration setpoint [rad/s²]");
        Kp.getOut().getSignal().setName("Proportional gain times the control error [rad/s²]");
        Kd.getOut().getSignal().setName("Derivative gain times the control error [rad/s²]");
        d1.getOut().getSignal().setName("Derivative of the control error [rad/s]");

         // Connect signals
        sum1.getIn(0).connect(in[0].getOut());
        sum1.getIn(1).connect(in[1].getOut());
        sum1.negateInput(1);
        Kp.getIn().connect(sum1.getOut());
        d1.getIn().connect(sum1.getOut());
        Kd.getIn().connect(d1.getOut());
        sum2.getIn(0).connect(Kp.getOut());
        sum2.getIn(1).connect(Kd.getOut());
        M.getIn().connect(sum2.getOut());
        d2.getIn().connect(in[1].getOut());

    }

};

#endif //CUSTOMBLOCKTEMPLATE_HPP_
