// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ma5951.utils.PhotonVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class AutoShelfAdjust extends CommandBase {
  /** Creates a new AutoShelfAdjust. */
  private SwerveDrivetrainSubsystem swerve;
  private PhotonVision photonVision;
  private PIDController pidX;
  private PIDController pidY;

  public AutoShelfAdjust() {
    photonVision = RobotContainer.photonVision;
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);
    pidX.setSetpoint(SwerveConstants.shelfSetPointX);
    pidY.setSetpoint(SwerveConstants.shelfSetPointY);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(
        pidX.calculate(photonVision.getDistanceToTargetMeters() *
            Math.cos(Math.toRadians(photonVision.getYaw()))),
        pidY.calculate(photonVision.getDistanceToTargetMeters() *
            Math.sin(Math.toRadians(photonVision.getYaw()))),
        swerve.getThetaPID().calculate(
            swerve.getFusedHeading(), SwerveConstants.shelfAngle),
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidX.atSetpoint() && pidY.atSetpoint() &&
        swerve.getThetaPID().atSetpoint();
  }
}
