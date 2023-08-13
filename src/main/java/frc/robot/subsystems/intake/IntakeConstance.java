package frc.robot.subsystems.intake;

public class IntakeConstance {    
    public static final double currentAmpThresholdCone = 33; // in amps
    public static final double currentAmpThresholdCude = 27; // in amps


    public static final double IntakePowerForCube = 0.6;
    public static final double IntakePowerForCone = -0.8;

    public static final double EjectPowerForCube = -0.4; // TODO
    public static final double EjectPowerForCubeForLow = -0.8; // TODO
    public static final double EjectPowerForCone = 0.8; // TODO

    public static final double ejectTime = 0.8; // TODO

    public static final double ElevatorEccalHoldeTime = 0.3 ; 
    // The time that the intake starts to holde the cone before the elavtor starts to accel **TODO**

    public static final double HoldConePower = -0.3; // TODO
    // The power to holde the cone while the elvator is accel **TODO**

}
