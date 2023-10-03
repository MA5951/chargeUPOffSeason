package frc.robot.commands.Automations.IntakeAutomations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeAutomation extends SequentialCommandGroup{
    public IntakeAutomation(double power) {
        addCommands(
                new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.4), 
                        new InstantCommand(() -> Intake.getInstance().setIgnoreCurrent(true)), 
                        new MotorCommand(Intake.getInstance(), power, power)
                    ),
                    new InstantCommand(() -> Intake.getInstance().setIgnoreCurrent(false)),
                    new MotorCommand(Intake.getInstance(), power, 0))
        );
    }
}
