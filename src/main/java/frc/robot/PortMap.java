package frc.robot;

public class PortMap {
    public static class Swerve {
        public static final int leftFrontAbsoluteEncoder = 22;
        public static final int leftFrontDriveID = 10;
        public static final int leftFrontTurningID = 5;

        public static final int leftBackAbsoluteEncoder = 24;
        public static final int leftBackDriveID = 2;
        public static final int leftBackTurningID = 3;

        public static final int rightFrontAbsoluteEncoder = 23;
        public static final int rightFrontDriveID = 7;
        public static final int rightFrontTurningID = 6;

        public static final int rightBackAbsoluteEncoder = 21;
        public static final int rightBackDriveID = 8;
        public static final int rightBackTurningID = 9;
    }

    public static class Elevator {
      public static final int masterMotorID = 0; //TODO
      public static final int slaveMotorID = 0;  //TODO
    }

    public static class Intake {
      public static final int intakeMotorID = 4; //TODO
      public static final int intakeLimitSwitchID = 0; //TODO
    }
}
