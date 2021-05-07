#include "MyRobotSafetyProperties.hpp"

MyRobotSafetyProperties::MyRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),
      //Levels
      slSystemOff("System is offline"),
      slShuttingDown("Shutting Down"),
      slHalting("Stop"),
      slStartingUp("Powering Up"),
      slEmergency("Emergency Mode"),
      slEmergencyBraking("Emergency Braking"),
      slSystemOn("System is online"),
      slMotorPowerOn("Activating Motors"),
      slSystemMoving("System is Moving"),

      // Events
      abort("Abort"),
      shutdown("Shutdown"),
      doSystemOn("Do system on"),
      systemStarted("System started"),
      emergency("Emergency"),
      resetEmergency("Reset emergency"),
      powerOn("Power on"),
      powerOff("Power off"),
      startMoving("Start moving"),
      stopMoving("Stop moving"),
      motorsHalted("Motors halted")
      
      
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare critical outputs
    // ... = hal.getLogicOutput("...");

    // criticalOutputs = { ... };

    // Declare critical inputs
    // ... = eeros::hal::HAL::instance().getLogicInput("...", ...);

    // criticalInputs = { ... };

    // Add all safety levels
    addLevel(slSystemOff);
    addLevel(slShuttingDown);
    addLevel(slHalting);
    addLevel(slStartingUp);
    addLevel(slEmergency);
    addLevel(slEmergencyBraking);
    addLevel(slSystemOn);
    addLevel(slMotorPowerOn);
    addLevel(slSystemMoving);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slStartingUp, kPublicEvent);
    slShuttingDown.addEvent(shutdown, slSystemOff, kPrivateEvent);
    slHalting.addEvent(motorsHalted, slShuttingDown, kPrivateEvent);
    slStartingUp.addEvent(systemStarted, slSystemOn, kPrivateEvent);
    slEmergency.addEvent(resetEmergency, slSystemOn, kPrivateEvent);
    slEmergencyBraking.addEvent(motorsHalted, slEmergency, kPrivateEvent);
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    slMotorPowerOn.addEvent(startMoving,slSystemMoving, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);
    slSystemMOving.addEvent(emergency, slEmergencyBraking, kPublicEvent);
    slSystemMoving.addEvent(abor, slHalting, kPublicEvent);

    // Add events to multiple safety levels
    // addEventToAllLevelsBetween(lowerLevel, upperLevel, event, targetLevel, kPublicEvent/kPrivateEvent);
    addEventToAllLevelBetween(slEmergency, slMotorPowerOn, abort, slShuttingDown, kPublicEvent);
    addEventToAllLevelBetween(slSystemOn, slMotorPowerOn, emergency, slEmergency, kPublicEvent);


    // Define input states and events for all levels
    // level.setInputActions({ ... });

    // Define output states and events for all levels
    // level.setOutputActions({ ... });

    // Define and add level functions
    slSystemOff.setLevelAction([&](SafetyContext *privateContext) {
        
        eeros::Executor::stop();
        eeros::sequencer::Sequencer::instance().abort();
    });
    
     slShuttingDown.setLevelAction([&](SafetyContext *privateContext) {
        
        cs.timedomain.stop();
        privateContext->triggerEvent(shutdown);
    });

    slHalting.setLevelAction([&](SafetyContext *privateContext) {
        
        // Check if motors are standing still
        privateContext->triggerEvent(motorsHalted);
    });

    slStartingUp.setLevelAction([&](SafetyContext *privateContext) {
        
        cs.timedomain.start();
        privateContext->triggerEvent(systemStarted);
    });

    slEmergency.setLevelAction([&](SafetyContext *privateContext) {
        
        if (slEmergency.getNofActivations()*dt ==1) // wait 1 sec
        {
            static int counter = 0;
            if (counter++ == 3) privateContext->triggerEvent(abort); //abort after entering emergency sequence 4 times
            privateContext->triggerEvent(resetEmergency);
        }
    });

    slEmergencyBraking.setLevelAction([&](SafetyContext *privateContext) {
        
        // Check if motors are standing still
        privateContext->triggerEvent(motorsHalted);
    });

    slSystemOn.setLevelAction([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(powerOn);
    });

    slMotorPowerOn.setLevelAction([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(startMoving);
    });

    slSystemMoving.setLevelAction([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(emergency);
    });

    // Define entry level
    setEntryLevel(slSystemOff);

    // Define exit function
    exitFunction = ([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(abort);
    });
}
