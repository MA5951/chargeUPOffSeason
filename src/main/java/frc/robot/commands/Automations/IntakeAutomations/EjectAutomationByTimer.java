package frc.robot.commands.Automations.IntakeAutomations;


import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstance;

public class EjectAutomationByTimer extends SequentialCommandGroup{
    
    public EjectAutomationByTimer() {
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(IntakeConstance.ejectTime),
                new MotorCommand(Intake.getInstance(), IntakeConstance.EjectPowerForCone , 0)),
            new InstantCommand(Intake.getInstance()::removeGamePieces)
        );
    }
}
