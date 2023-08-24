package frc.robot.subsystems.elevator;

public class ElevatorConstance {    
    public static final double kP = 3.3;
    public static final double kI = 0;
    public static final double kD = 6;

    public static final double closedLoopRampRate = 0.25;

    public static final double gear = 70d / 9d;
    public static final double raduis = 0.021;

    public static final double positionConversionFactor = ((2 * Math.PI * raduis)
     / gear) * 1.11111111111111111111;

    public static final double tolerance = 0.03;

    public static final double maxPose = 1.6;
    public static final double minPose = -0.01;

    public static final double lowPose = 0; // TODO
    public static final double highPose = 1.6;
    public static final double ConeMidPose = 0; //TODO
    public static final double CubeMidPose = 0; //TODO

    public static final double ShelfPose = 0; // TODO
    public static final int kCPR = 4096;

    public static final double lowerPower = -0.3; // TODO
    public static final double currentAmpThreshold = 25; // TODO

}
