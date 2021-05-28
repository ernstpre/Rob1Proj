#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"), E2("enc2"),
      myConstant(0.2), myConstant2(0.0),
      invMotMod(AMRSC::MOT::QMax, AMRSC::MOT::qdMax, AMRSC::MOT::i, AMRSC::MOT::kM, AMRSC::MOT::R),
      piController(1.0 / dt, AMRSC::CONT::D, AMRSC::CONT::s, AMRSC::CONT::M, AMRSC::CONT::ILIMIT),
      M1("motor1"),
      M2("motor2"),
      timedomain("Main time domain", dt, true),
      
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
      R2(8.0)
      
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    d1.setName("d1");
    d2.setName("d2");
    myConstant.setName("My constant");
    piController.setName("PI Controller");
    invMotMod.setName("InvMotMod");
    M1.setName("M1");
    M2.setName("M2");


    // Name all signals
   
    E1.getOut().getSignal().setName("Position encoder1 [m]");
    E2.getOut().getSignal().setName("Position encoder2 [m]");
    myConstant.getOut().getSignal().setName("Right motor voltage [V]");
    
    // Connect signals
    d1.getIn().connect(E1.getOut());
    piController.getInqds().connect(myConstant.getOut());
    piController.getInqd().connect(d1.getOut());
    invMotMod.getInQ().connect(piController.getOutQ());
    invMotMod.getInqd().connect(piController.getOutqd());
    M1.getIn().connect(invMotMod.getOutU());
    M2.getIn().connect(invMotMod.getOutU());
    
    // controller1.getIn(0).connect(E2.getOut());
    // controller1.getIn(1).connect(E1.getOut());
    // QMax1.getIn().connect(controller1.getOut(0));
    // i1_inv.getIn().connect(QMax1.getOut());
    // kM1_inv.getIn().connect(i1_inv.getOut());
    // R1.getIn().connect(kM1_inv.getOut());

    // d1.getIn().connect(E1.getOut());
    // qdMax1.getIn().connect(d1.getOut());
    // i1.getIn().connect(qdMax1.getOut());
    // kM1.getIn().connect(i1.getOut());

    // U1.getIn(0).connect(R1.getOut());
    // U1.getIn(1).connect(kM1.getOut());
    // M1.getIn().connect(U1.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(d1);
    timedomain.addBlock(myConstant);
    timedomain.addBlock(myConstant2);
    timedomain.addBlock(piController);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);
   

    // timedomain.addBlock(E1);
    // timedomain.addBlock(E2);
    // timedomain.addBlock(controller1);
    // timedomain.addBlock(controller2);
    // timedomain.addBlock(QMax1);
    // timedomain.addBlock(QMax2);
    // timedomain.addBlock(qdMax1);
    // timedomain.addBlock(qdMax2);
    // timedomain.addBlock(i1);
    // timedomain.addBlock(i2);
    // timedomain.addBlock(i1_inv);
    // timedomain.addBlock(i2_inv);
    // timedomain.addBlock(kM1);
    // timedomain.addBlock(kM2);
    // timedomain.addBlock(kM1_inv);
    // timedomain.addBlock(kM2_inv);
    // timedomain.addBlock(R1);
    // timedomain.addBlock(R2);
    // timedomain.addBlock(d1);
    // timedomain.addBlock(d2);
    // timedomain.addBlock(U1);
    // timedomain.addBlock(U2);
    // timedomain.addBlock(M1);
    // timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}