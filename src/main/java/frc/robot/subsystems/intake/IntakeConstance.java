package frc.robot.subsystems.intake;

public class IntakeConstance {    
    public static final double currentAmpThreshold = 35;

    public static final double IntakePowerForCube = -0.5;
    public static final double IntakePowerForCone = 0.8;

    public static final double EjectPowerForCube = 0.8;
    public static final double EjectPowerForCubeForLow = 0.9;
    public static final double EjectPowerForCone = -0.8;

    public static final double ejectTime = 0.4;

    public static final double ElevatorEccalHoldeTime = 0.1 ; 
    // The time that the intake starts to holde the cone before the elavtor starts to accel **TODO**

    public static final double HoldConePower = 0.2;
    // The power to holde the cone while the elvator is accel **TODO**

}
