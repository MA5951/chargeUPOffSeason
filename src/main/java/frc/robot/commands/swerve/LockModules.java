// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class LockModules extends CommandBase {
  /** Creates a new LockModules. */
  private SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
  private final SwerveModuleState[] states;
  public LockModules() {
    swerveDrivetrainSubsystem = SwerveDrivetrainSubsystem.getInstance();
    addRequirements(swerveDrivetrainSubsystem);
    states = new SwerveModuleState[] {
      new SwerveModuleState(
        0.00000000000000000000000000000000000000001,
        Rotation2d.fromDegrees(45)
      ),
      new SwerveModuleState(
        0.00000000000000000000000000000000000000001,
        Rotation2d.fromDegrees(-45)
      ),
      new SwerveModuleState(
        0.00000000000000000000000000000000000000001,
        Rotation2d.fromDegrees(-45)
      ),
      new SwerveModuleState(
        0.00000000000000000000000000000000000000001,
        Rotation2d.fromDegrees(45)
      ),
    };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrivetrainSubsystem.setModules(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrainSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
