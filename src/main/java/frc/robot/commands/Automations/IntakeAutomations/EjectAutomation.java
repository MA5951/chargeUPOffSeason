package frc.robot.commands.Automations.IntakeAutomations;

import com.ma5951.utils.commands.MotorCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstance;

public class EjectAutomation extends SequentialCommandGroup{
    private static double setPower() {
        return Intake.getInstance().isCubeInIntake() ? IntakeConstance.EjectPowerForCube
            : IntakeConstance.EjectPowerForCone;
    }
    public EjectAutomation() {
        addCommands(
            new MotorCommand(Intake.getInstance(), EjectAutomation::setPower , 0)
        );
    }
}
