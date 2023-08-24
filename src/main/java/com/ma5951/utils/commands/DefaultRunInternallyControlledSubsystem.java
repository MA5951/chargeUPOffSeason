// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.commands;

import com.ma5951.utils.subsystem.DefaultInternallyControlledSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultRunInternallyControlledSubsystem extends CommandBase {
  /** Creates a new DefultControlCommandInSubsystemControl. */
  private DefaultInternallyControlledSubsystem subsystem;
  private double oldSetPoint;

  public DefaultRunInternallyControlledSubsystem(
    DefaultInternallyControlledSubsystem subsystem) {
      this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldSetPoint = subsystem.getSetPoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (subsystem.canMove()) {
      oldSetPoint = subsystem.getSetPoint();
      subsystem.calculate(subsystem.getSetPoint());
    } else {
      subsystem.calculate(oldSetPoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
