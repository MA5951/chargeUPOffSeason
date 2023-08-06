// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Camera {
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_X = 0; //TODO
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Y = 0; //TODO
        public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Z = 0; //TODO
        public static final double CAMERA_ROLL = 0; //TODO
        public static final double CAMERA_PITCH = 0; //TODO
        public static final double CAMERA_YAW = 0; //TODO
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
        public static final double TO_2 = 0.56 + 0.1;
        public static final double TO_3 = 1.08 + 0.1;
        private static final double BLUE_X = 0.95 + Robot.WIDTH / 2;
        private static final double RED_X = FIELD_LENGTH_METERS - 1.5 - Robot.WIDTH / 2;
        private static final double A1Y = Robot.LENGTH / 2.0 + 0.1;
        private static final double A2Y = Robot.LENGTH / 2.0 + TO_2;
        private static final double A3Y = Robot.LENGTH / 2.0 + TO_3;
        private static final double DISTANCE_FROM_GRIDS = A3Y - A2Y + 0.04;
        private static final double B1Y = A3Y + DISTANCE_FROM_GRIDS;
        private static final double B2Y = B1Y + DISTANCE_FROM_GRIDS;
        private static final double B3Y = B2Y + DISTANCE_FROM_GRIDS;
        private static final double C1Y = B3Y + DISTANCE_FROM_GRIDS;
        private static final double C2Y = C1Y + DISTANCE_FROM_GRIDS;
        private static final double C3Y = C2Y + DISTANCE_FROM_GRIDS;
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
