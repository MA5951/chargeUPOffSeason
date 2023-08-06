// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.PhotonVision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public static final CommandPS4Controller DRIVER_PS4_CONTROLLER = 
    new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public static final CommandPS4Controller OPERATOR_PS4_CONTROLLER =
    new CommandPS4Controller(OperatorConstants.OPERATOR_CONTROLLER_PORT);   

  public static PhotonVision photonVision;
  private static AprilTagFieldLayout aprilTagFieldLayout;

  public RobotContainer() {
    // Configure the trigger bindings
    try {
      aprilTagFieldLayout = 
        AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.err.println(e);
    }

    photonVision  = new PhotonVision(
      "ma5951",
      new Transform3d(
       new Translation3d(
        Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_X,
        Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_Y,
        Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_Z
       ), new Rotation3d(
        Constants.Camera.CAMERA_ROLL,
        Constants.Camera.CAMERA_PITCH,
        Constants.Camera.CAMERA_YAW
       )),
      aprilTagFieldLayout
       );

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
