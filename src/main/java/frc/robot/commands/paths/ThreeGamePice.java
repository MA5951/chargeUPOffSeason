// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.ElevatorAutomations.ResetElevator;
import frc.robot.commands.Automations.IntakeAutomations.EjectAutomationByTimer;
import frc.robot.commands.Automations.IntakeAutomations.RunIntakeAutomation;
import frc.robot.commands.ScoringAutomation.EjectAutomationAuto;
import frc.robot.subsystems.elevator.ElevatorConstance;
import frc.robot.subsystems.intake.IntakeConstance;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem.Auto;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeGamePice extends SequentialCommandGroup {
  /** Creates a new TwoPiceAuto. */
  public ThreeGamePice() {
    addCommands(
      new InstantCommand(() ->SwerveDrivetrainSubsystem.getInstance().setAuto(Auto.ThreeGamePice)),  
    new ResetElevator(),     
    new EjectAutomationAuto(ElevatorConstance.highPoseCone),
        new ParallelCommandGroup(
            SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Three game pcs 1", true , 4 , 3),
            new RunIntakeAutomation(IntakeConstance.IntakePowerForCone)),
        SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Three game pcs 2", false , 4 , 3),
        new EjectAutomationByTimer(),
        new ParallelCommandGroup(
            SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Three game pcs 3", false , 4 , 3),
            new RunIntakeAutomation(IntakeConstance.IntakePowerForCube)));
  }
}