// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static final Alliance alliance = Alliance.Red;

        public static class Camera {
                public static final double CAMERA_DISTANCE_FROM_CENTER_IN_X = alliance == Alliance.Blue ? -0.2 : 0.4; // TODO

                public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Y = alliance == Alliance.Blue ? 0.434 : 0.41; // TODO

                public static final double CAMERA_DISTANCE_FROM_CENTER_IN_Z = alliance == Alliance.Blue ? 0.7 : 0.7; // TODO

                public static final double CAMERA_ROLL = alliance == Alliance.Blue ? 0 : 0; // TODO

                public static final double CAMERA_PITCH = alliance == Alliance.Blue ? 0 : 0; // TODO

                public static final double CAMERA_YAW = alliance == Alliance.Blue ? 0 : 0; // TODO
        }

        public static class ColorPresets {
                public static final Color CONE_YELLOW = new Color(255, 237, 70);
                public static final Color CUBE_PURPLE = new Color(113, 82, 199);
                public static final Color RED = new Color(216, 24, 24);
                public static final Color BLUE = new Color(31, 55, 178);
                public static final Color WHITE = new Color(255, 255, 255);
                public static final Color BLACK = new Color(0, 0, 0);
                public static final Color GREEN = new Color(47, 188, 52);
                public static final Color ORANGE = new Color(246, 167, 48);
                public static final Color CYAN = new Color(51, 204, 204);
                public static final Color MAcolor = new Color(127, 0, 0);
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
                private static final double BLUE_X = 1.4;
                private static final double RED_X = 15;
                private static final double A1Y = 0.54;
                private static final double A2Y = 1.09;
                private static final double A3Y = 1.66;
                private static final double B1Y = 2.12;
                private static final double B2Y = 2.77;
                private static final double B3Y = 3.32;
                private static final double C1Y = 3.91;
                private static final double C2Y = 4.45;
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
