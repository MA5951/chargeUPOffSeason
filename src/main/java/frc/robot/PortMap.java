package frc.robot;

public class PortMap {
    public static class Swerve {
        public static final int leftFrontAbsoluteEncoder = 22;
        public static final int leftFrontDriveID = 4;
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
      public static final int masterMotorID = 10;
      public static final int slaveMotorID = 11;
    }

    public static class Intake {
      public static final int intakeMotorID = 12;
      public static final int sensorID = 0;
    }
}