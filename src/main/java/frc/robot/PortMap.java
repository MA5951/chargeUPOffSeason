package frc.robot;

public class PortMap {
    public static class Swerve {
        public static final int leftFrontAbsoluteEncoder = 21; // TODO
        public static final int leftFrontDriveID = 2; // TODO
        public static final int leftFrontTurningID = 3; // TODO

        public static final int leftBackAbsoluteEncoder = 22; // TODO
        public static final int leftBackDriveID = 4; // TODO
        public static final int leftBackTurningID = 5; // TODO

        public static final int rightFrontAbsoluteEncoder = 23; // TODO
        public static final int rightFrontDriveID = 7; // TODO
        public static final int rightFrontTurningID = 6; // TODO

        public static final int rightBackAbsoluteEncoder = 24; // TODO
        public static final int rightBackDriveID = 8; // TODO
        public static final int rightBackTurningID = 9; // TODO
    }

    public static class Elevator {
      public static final int masterMotorID = 0; //TODO
      public static final int slaveMotorID = 0;  //TODO
    }

    public static class Intake {
      public static final int intakeMotorID = 4; //TODO
      public static final int intakeLimitSwitchID = 0; //TODO
    }

    public static class Led {
      public static final int ledPort = 2; //TODO
      public static final int ledPort2 = 7; //TODO
    }
}
