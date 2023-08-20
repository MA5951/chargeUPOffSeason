package frc.robot.subsystems.elevator;

import com.ma5951.utils.RobotConstants;

public class ElevatorConstance {    
    public static final double kP = 0; // TODO
    public static final double kI = 0; // TODO
    public static final double kD = 0; // TODO

    public static final double kV = 473;
    public static final double kT = (60 / (2 * Math.PI * kV));

    public static final double mass = 10;
    public static final double gear = 8;
    public static final double raduis = 0.21;
    public static final double angle = Math.toRadians(46.28);

    public static final double resistance = 0; // TODO voltage / current *Need to check once*

    public static final double kG = (((mass * RobotConstants.KGRAVITY_ACCELERATION
        * Math.sin(angle) * raduis) / (kT * gear)) * resistance) / 12d;
        // TODO need to check

    public static final double positionConversionFactor = 2 * Math.PI * raduis;

    public static final double tolerance = 0; // TODO

    public static final double maxPose = 1.6;
    public static final double minPose = 0;

    public static final double lowPose = 0; // TODO
    public static final double highPose = 1.6;
    public static final double ConeMidPose = 0; //TODO
    public static final double CubeMidPose = 0; //TODO

    public static final double ShelfPose = 0; // TODO
    public static final int kCPR = 4096;
}
