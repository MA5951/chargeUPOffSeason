// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations.ElevatorAutomations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstance;

public class RunElevatorByCurrent extends CommandBase {
  /** Creates a new RunElevatorByCurrent. */
  private final Command command;

  public RunElevatorByCurrent() {
    command = new MotorCommand(
      Elevator.getInstance(), ElevatorConstance.lowerPower, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Elevator.getInstance().resetPose(0);
    command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ElevatorConstance.currentAmpThreshold < Elevator.getInstance().getCurrent();
  }
}
