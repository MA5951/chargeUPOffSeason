// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations.TeleopAutomations;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Automations.IntakeAutomations.RunIntakeAutomation;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElvatoreIntakeAutomation extends SequentialCommandGroup {
  /** Creates a new ShelfIntakeAutomation. */
  public ElvatoreIntakeAutomation(double power , double hight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new InstantCommand(Intake.getInstance()::removeGamePieces),  
    new InstantCommand(() -> Elevator.getInstance().setSetPoint(hight))
      .andThen(new RunIntakeAutomation(power))
    );
  }
}
