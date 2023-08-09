package frc.robot.subsystems.elevator;

import com.ma5951.utils.RobotConstants;

public class ElevatorConstance {    
    public static final double kP = 0; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 0; // TODO

    public static final double kV = 473;
    public static final double kT = (60 / (2 * Math.PI * kV));

    public static final double mass = 0; // TODO
    public static final double gear = 0; // TODO
    public static final double raduis = 0; //TODO
    public static final double angle = 0; // TODO radians

    public static final double kG = (mass * RobotConstants.KGRAVITY_ACCELERATION
        * Math.sin(angle) * raduis) / (kT * gear); // TODO need to check

    public static final double positionConversionFactor = 0; // TODO

    public static final double tolerance = 0; // TODO

    public static final double maxPose = 0; // TODO
    public static final double minPose = 0; // TODO

    public static final int kCPR = 4096;
}
