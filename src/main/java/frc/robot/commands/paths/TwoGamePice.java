// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.paths;

import com.ma5951.utils.commands.MotorCommand;
import com.ma5951.utils.commands.RunInternallyControlledSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Automations.ElevatorAutomations.ResetElevator;
import frc.robot.commands.Automations.IntakeAutomations.RunIntakeAutomation;
import frc.robot.commands.ScoringAutomation.EjectAutomationAuto;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstance;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstance;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoGamePice extends SequentialCommandGroup {
  /** Creates a new OnePiceAuto. */
  public TwoGamePice() {
    addCommands(
        new ResetElevator(),     
    new EjectAutomationAuto(ElevatorConstance.highPoseCone),
        new ParallelCommandGroup(
            SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Two game pcs 1", true , 4.5 , 3),
            new RunIntakeAutomation(IntakeConstance.IntakePowerForCone)),
        SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Two game pcs 2", false , 4.5 , 3),
        new InstantCommand(() -> Intake.getInstance().setCubeState(false)),
        new ParallelDeadlineGroup(
            new WaitCommand(0.3),
            new MotorCommand(Intake.getInstance(), IntakeConstance.HoldConePower, IntakeConstance.HoldConePower)),
        new ParallelDeadlineGroup(
            new RunInternallyControlledSubsystem(
                Elevator.getInstance(), ElevatorConstance.highPoseCone, true),
            new MotorCommand(Intake.getInstance(), IntakeConstance.HoldConePower, 0)),
        new EjectAutomationAuto(ElevatorConstance.highPoseCone));
        //SwerveDrivetrainSubsystem.getInstance().getAutonomousPathCommand("Two game pcs 3", false , 4.96 , 3));

  }
}
