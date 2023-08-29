// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringAutomation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.ElevatorAutomations.SetElvator;
import frc.robot.commands.Automations.IntakeAutomations.EjectAutomationByTimer;
import frc.robot.subsystems.elevator.ElevatorConstance;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectAutomationAuto extends SequentialCommandGroup {
  /** Creates a new EjectAutomationAuto. */
  public EjectAutomationAuto(double scoring_pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Intake.getInstance().setCubeState(false)),
      new SetElvator(scoring_pose),
      new EjectAutomationByTimer(),
      new SetElvator(ElevatorConstance.minPose)
    );
  }
}
