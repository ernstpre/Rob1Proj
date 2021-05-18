#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"), E2("enc2"),
      controller1(1.0 / dt, 0.7, 4.4, 6.8e-8 * 3441.0 / 104.0 * 3441.1 / 104.0),
      controller2(1.0 / dt, 0.7, 4.4, 6.8e-8 * 3441.0 / 104.0 * 3441.1 / 104.0),
      QMax1(0.1), QMax2(0.1),
      qdMax1(21.3), qdMax2(21.3),
      i1(3441.0 / 104.0),
      i2(3441.0 / 104.0),
      i1_inv(104.0 / 3441.0),
      i2_inv(104.0 / 3441.0),
      kM1(8.44e-3),
      kM2(8.44e-3),
      kM1_inv(1 / 8.44e-3),
      kM2_inv(1 / 8.44e-3),
      R1(8.0),
      R2(8.0),
      M1("motor1"),
      M2("motor2"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    controller1.setName("controller1");
    controller2.setName("controller2");
    QMax1.setName("QMax1");
    QMax2.setName("QMax2");
    qdMax1.setName("qdMax1");
    qdMax2.setName("qdMax2");
    i1.setName("i1");
    i2.setName("i2");
    i1_inv.setName("i1_inv");
    i2_inv.setName("i2_inv");
    kM1.setName("kM1");
    kM2.setName("kM2");
    kM1_inv.setName("kM1_inv");
    kM2_inv.setName("kM2_inv");
    R1.setName("R1");
    R2.setName("R2");
    d1.setName("d1");
    d2.setName("d2");
    U1.setName("U1");
    U2.setName("U2");
    M1.setName("M1");
    M2.setName("M2");

    // Name all signals
    E1.getOut().getSignal().setName("Position encoder1 [rad]");
    E2.getOut().getSignal().setName("Position encoder 2 [rad]");
    controller1.getOut(0).getSignal().setName("Output shaft torque setpoint 1 [Nm]");
    controller2.getOut(0).getSignal().setName("Output shaft torque setpoint 2 [Nm]");
    QMax1.getOut().getSignal().setName("Saturated output shaft torque setopint 1 [Nm]");
    QMax2.getOut().getSignal().setName("Saturated output shaft torque setopint 2 [Nm]");
    qdMax1.getOut().getSignal().setName("Saturated output shaft velocity setpoint 1 [rad/s]");
    qdMax2.getOut().getSignal().setName("Saturated output shaft velocity setpoint 2 [rad/s]");
    i1.getOut().getSignal().setName("Motor 1 velocity setpoint [rad/s]");
    i2.getOut().getSignal().setName("Motor 2 velocity setpoint [rad/s]");
    i1_inv.getOut().getSignal().setName("Motor 1 torque setpoint [Nm]");
    i2_inv.getOut().getSignal().setName("Motor 2 torque setpoint [Nm]");
    kM1.getOut().getSignal().setName("Motor 1 setpoint voltage [V]");
    kM2.getOut().getSignal().setName("Motor 2 setpoint voltage [V]");
    kM1_inv.getOut().getSignal().setName("Motor 1 setpoint current [A]");
    kM2_inv.getOut().getSignal().setName("Motor 2 setpoint current [A]");
    R1.getOut().getSignal().setName("Motor 1 setpoint voltage from pd controller [V]");
    R2.getOut().getSignal().setName("Motor 2 setpoint voltage from pd controller [V]");
    d1.getOut().getSignal().setName("Output shaft velocity setpoint 1 [rad/s]");
    d2.getOut().getSignal().setName("Output shaft velocity setpoint 2 [rad/s]");
    U1.getOut().getSignal().setName("Motor 1 setpoint voltage [V]");
    U2.getOut().getSignal().setName("Motor 2 setpoint voltage [V]");


    // Connect signals
    controller1.getIn(0).connect(E2.getOut());
    controller1.getIn(1).connect(E1.getOut());
    QMax1.getIn().connect(controller1.getOut(0));
    i1_inv.getIn().connect(QMax1.getOut());
    kM1_inv.getIn().connect(i1_inv.getOut());
    R1.getIn().connect(kM1_inv.getOut());

    d1.getIn().connect(E1.getOut());
    qdMax1.getIn().connect(d1.getOut());
    i1.getIn().connect(qdMax1.getOut());
    kM1.getIn().connect(i1.getOut());

    U1.getIn(0).connect(R1.getOut());
    U1.getIn(1).connect(kM1.getOut());
    M1.getIn().connect(U1.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(controller1);
    timedomain.addBlock(controller2);
    timedomain.addBlock(QMax1);
    timedomain.addBlock(QMax2);
    timedomain.addBlock(qdMax1);
    timedomain.addBlock(qdMax2);
    timedomain.addBlock(i1);
    timedomain.addBlock(i2);
    timedomain.addBlock(i1_inv);
    timedomain.addBlock(i2_inv);
    timedomain.addBlock(kM1);
    timedomain.addBlock(kM2);
    timedomain.addBlock(kM1_inv);
    timedomain.addBlock(kM2_inv);
    timedomain.addBlock(R1);
    timedomain.addBlock(R2);
    timedomain.addBlock(d1);
    timedomain.addBlock(d2);
    timedomain.addBlock(U1);
    timedomain.addBlock(U2);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}