package frc.robot.commands.Automations.IntakeAutomations;

import java.util.function.Supplier;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;


public class EjectAutomation extends SequentialCommandGroup{
    public EjectAutomation(Supplier<Double> getPower) {
        addCommands(
            new MotorCommand(Intake.getInstance(), getPower , 0)
        );
    }
}
