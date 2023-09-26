// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.ma5951.utils.PhotonVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class SimpleAutoAdjust extends CommandBase {
  /** Creates a new AutoShelfAdjust. */
  private SwerveDrivetrainSubsystem swerve;
  private PhotonVision photonVision;
  private PIDController pidX;
  private PIDController pidY;
  private double thetaSetPoint;

  public SimpleAutoAdjust(
      double xSetPoint, double ySetPoint, double thetaSetPoint) {
    photonVision = RobotContainer.photonVision;
    swerve = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerve);
    pidX = new PIDController(
        SwerveConstants.x_KP, SwerveConstants.x_KI, SwerveConstants.x_KD);
    pidY = new PIDController(
        SwerveConstants.y_KP, SwerveConstants.y_KI, SwerveConstants.y_KD);
    pidX.setSetpoint(xSetPoint);
    pidY.setSetpoint(ySetPoint);
    this.thetaSetPoint = thetaSetPoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance;
    
    if (photonVision.getPipeline() == Constants.pipline.apriltag) {
      distance = photonVision.getDistanceToTargetMeters();
      

    } else {
      distance = photonVision.getDistanceToTargetMeters(
          Constants.FieldConstants.reflectiveHight);

      
    }
    double angle = Math.toRadians(photonVision.getYaw());

    

    swerve.drive(
        -pidX.calculate( 1 * distance * Math.cos(angle)),
        pidY.calculate(distance * Math.sin(angle)),
        SwerveDrivetrainSubsystem.getInstance().getThetaPID().calculate(
            SwerveDrivetrainSubsystem.getInstance().getPose().getRotation().getRadians(), thetaSetPoint),
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
    return !photonVision.hasTarget();
  }
}
