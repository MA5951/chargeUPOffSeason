// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class AutoAdjustForScore extends CommandBase {
  /** Creates a new AutoAdjustForScore. */
  private Command GoToScore;

  public AutoAdjustForScore() {
    addRequirements(SwerveDrivetrainSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    GoToScore = SwerveDrivetrainSubsystem.getInstance().getTelopPathCommand();
    GoToScore.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    GoToScore.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveDrivetrainSubsystem.getInstance().stop();
    GoToScore.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return GoToScore.isFinished();
  }
}
