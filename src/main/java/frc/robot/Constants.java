// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Alliance alliance = Alliance.Blue;

    public static class Camera {
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_X = 
            alliance == Alliance.Blue ? 0 : 0; //TODO
            
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Y =
            alliance == Alliance.Blue ? 0 : 0; //TODO

        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Z =
            alliance == Alliance.Blue ? 0 : 0; //TODO

        public static final double CAMERA_ROLL =
            alliance == Alliance.Blue ? 0 : 0; //TODO

        public static final double CAMERA_PITCH =
            alliance == Alliance.Blue ? 0 : 0; //TODO

        public static final double CAMERA_YAW = 
            alliance == Alliance.Blue ? 0 : 0; //TODO
    }
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class Robot {
        public final static double LENGTH = 0.915;
        public final static double WIDTH = 0.915;
    }

    public static final class FieldConstants {
        public static final double FIELD_WIDTH_METERS = 8.02;
        public static final double FIELD_LENGTH_METERS = 16.54;
        private static final double BLUE_X = 0.143 + Robot.LENGTH / 2d;
        private static final double RED_X = FieldConstants.FIELD_WIDTH_METERS - BLUE_X;
        private static final double A1Y = 0.51;
        private static final double A2Y = 1.06;
        private static final double A3Y = 1.63;
        private static final double B1Y = 2.19;
        private static final double B2Y = 2.74;
        private static final double B3Y = 3.31;
        private static final double C1Y = 3.87;
        private static final double C2Y = 4.42;
        private static final double C3Y = 4.98;
        public static final Translation2d[] ScoringPoses = {
            new Translation2d(
                BLUE_X, C3Y),
            new Translation2d(
                BLUE_X, C2Y),
            new Translation2d(
                BLUE_X, C1Y),
            new Translation2d(
                BLUE_X, B3Y),
            new Translation2d(
                BLUE_X, B2Y),
            new Translation2d(
                BLUE_X, B1Y),
            new Translation2d(
                BLUE_X, A3Y),
            new Translation2d(
                BLUE_X, A2Y),
            new Translation2d(
                BLUE_X, A1Y),
            new Translation2d(
                RED_X, C3Y),
            new Translation2d(
                RED_X, C2Y),
            new Translation2d(
                RED_X, C1Y),
            new Translation2d(
                RED_X, B3Y),
            new Translation2d(
                RED_X, B2Y),
            new Translation2d(
                RED_X, B1Y),
            new Translation2d(
                RED_X, A3Y),
            new Translation2d(
                RED_X, A2Y),
            new Translation2d(
                RED_X, A1Y)
        }; // need to check TODO
    }
}
