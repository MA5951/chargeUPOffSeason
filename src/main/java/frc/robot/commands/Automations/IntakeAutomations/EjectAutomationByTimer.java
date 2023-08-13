package frc.robot.commands.Automations.IntakeAutomations;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstance;

public class EjectAutomationByTimer extends SequentialCommandGroup{
    public EjectAutomationByTimer(double power) {
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(IntakeConstance.ejectTime),
                new EjectAutomation(() -> power)),
            new InstantCommand(Intake.getInstance()::removeGamePieces)
        );
    }
}
