// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.PhotonVision;
import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Automations.ElevatorAutomations.ResetElevator;
import frc.robot.commands.Automations.ElevatorAutomations.SetElvator;
import frc.robot.commands.Automations.IntakeAutomations.EjectAutomation;
import frc.robot.commands.Automations.IntakeAutomations.RunIntakeAutomation;
import frc.robot.commands.Automations.TeleopAutomations.ShelfIntakeAutomation;
import frc.robot.commands.ScoringAutomation.EjectAutomationAuto;
import frc.robot.commands.swerve.AutoAdjustForScore;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstance;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstance;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.commands.paths.CenterToClimb;
import frc.robot.commands.Automations.ElevatorAutomations.ResetElevator;

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

  // public static PhotonVision photonVision;
  // private static AprilTagFieldLayout aprilTagFieldLayout;

  public RobotContainer() {
    // Configure the trigger bindings
    // try {
    //   aprilTagFieldLayout = 
    //     AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    // } catch (Exception e) {
    //   System.err.println(e);
    // }

    // photonVision  = new PhotonVision(
    //   "ma5951",
    //   new Transform3d(
    //    new Translation3d(
    //     Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_X,
    //     Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_Y,
    //     Constants.Camera.CAMERA_DISTANCE_FROM_CENTER_IN_Z
    //    ), new Rotation3d(
    //     Constants.Camera.CAMERA_ROLL,
    //     Constants.Camera.CAMERA_PITCH,
    //     Constants.Camera.CAMERA_YAW
    //    )),
    //   aprilTagFieldLayout
    //    );

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
    // r1 = cube intake, l1 = cone intake, squere = no timer eject, l2 = timer eject, r2 = no timer eject
    DRIVER_PS4_CONTROLLER.R1().whileTrue(
      new InstantCommand(() -> Intake.getInstance().setIgnoreSensor(true))
      .andThen(new RunIntakeAutomation(IntakeConstance.IntakePowerForCone)));

    DRIVER_PS4_CONTROLLER.L1().whileTrue(
      new InstantCommand(() -> Intake.getInstance().setIgnoreSensor(false))
      .andThen(new RunIntakeAutomation(IntakeConstance.IntakePowerForCube)));

    DRIVER_PS4_CONTROLLER.circle().whileTrue(new EjectAutomation())
      .whileFalse(new InstantCommand(Intake.getInstance()::removeGamePieces)
      .andThen(new InstantCommand(() -> Elevator.getInstance().setSetPoint(ElevatorConstance.minPose))));
    
    DRIVER_PS4_CONTROLLER.square().whileTrue(
      new MotorCommand(Intake.getInstance(), IntakeConstance.EjectPowerForCubeForLow, 0))
          .whileFalse(new InstantCommand(Intake.getInstance()::removeGamePieces));

    DRIVER_PS4_CONTROLLER.triangle().whileTrue(
      new InstantCommand(() -> SwerveDrivetrainSubsystem.getInstance().updateOffset())
    );

    DRIVER_PS4_CONTROLLER.R2().whileTrue(
      new InstantCommand(
        () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(0.4))
    ).whileFalse(
      new InstantCommand(
        () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(1))
    );

    DRIVER_PS4_CONTROLLER.L2().whileTrue(
      new InstantCommand(
        () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(0.1))
    ).whileFalse(
      new InstantCommand(
        () -> SwerveDrivetrainSubsystem.getInstance().FactorVelocityTo(1))
    );

    DRIVER_PS4_CONTROLLER.touchpad().whileTrue(
      new MotorCommand(Intake.getInstance(), IntakeConstance.IntakePowerForCone, 0)
    );

    // DRIVER_PS4_CONTROLLER.cross().whileTrue(new AutoAdjustForScore());

    DRIVER_PS4_CONTROLLER.povDown().whileTrue(new ResetElevator());

    OPERATOR_PS4_CONTROLLER.circle().whileTrue(new ResetElevator());

    OPERATOR_PS4_CONTROLLER.triangle().whileTrue(
      new InstantCommand(() -> Intake.getInstance().setIgnoreSensor(true))
      .andThen(new ShelfIntakeAutomation(IntakeConstance.IntakePowerForCone))
      ).whileFalse(
        new SetElvator(ElevatorConstance.minPose)
      );

      OPERATOR_PS4_CONTROLLER.square().whileTrue(
        new InstantCommand(() -> Intake.getInstance().setIgnoreSensor(false))
        .andThen(new ShelfIntakeAutomation(IntakeConstance.IntakePowerForCube))
      ).whileFalse(
        new InstantCommand(() -> Elevator.getInstance().setSetPoint(ElevatorConstance.minPose))
      );

      OPERATOR_PS4_CONTROLLER.povUp().whileTrue(
        new SetElvator(Elevator.getInstance().highHight)
      );
  
      OPERATOR_PS4_CONTROLLER.povDown().whileTrue(
        new SetElvator(ElevatorConstance.lowPose)
      );
  
      OPERATOR_PS4_CONTROLLER.povRight().whileTrue(
        new SetElvator(Elevator.getInstance().midhight)
      );

      OPERATOR_PS4_CONTROLLER.povLeft().whileTrue(
        new SetElvator(Elevator.getInstance().minHight)
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ResetElevator().andThen(new CenterToClimb());
  }
}
