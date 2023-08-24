// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.IntakeAutomations.RunIntakeAutomation;
import frc.robot.commands.ScoringAutomation.EjectAutomationAuto;
import frc.robot.subsystems.elevator.ElevatorConstance;
import frc.robot.subsystems.intake.IntakeConstance;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiceAuto extends SequentialCommandGroup {
  /** Creates a new TwoPiceAuto. */
  public TwoPiceAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new EjectAutomationAuto(ElevatorConstance.maxPose, true),
      new ParallelCommandGroup(
        SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Two game pcs 1"),
        new RunIntakeAutomation(IntakeConstance.IntakePowerForCone)
      ),
      SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Two game pcs 2"),
      new EjectAutomationAuto(ElevatorConstance.maxPose, true)
    );
  }
}
