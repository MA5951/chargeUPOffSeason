package frc.robot.commands.paths;




import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Automations.ElevatorAutomations.ResetElevator;
import frc.robot.commands.ScoringAutomation.EjectAutomationAuto;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.LockModules;
import frc.robot.subsystems.elevator.ElevatorConstance;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.Leds.Autostate;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;


public class ClimeCommunity extends SequentialCommandGroup {

        private static SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();

        private static boolean isAtClimbAngle() {
                return Math.abs(swerve.getPitch()) > SwerveConstants.ANGLE_BEFORE_CLIMB;
        }

        public ClimeCommunity() {

                addCommands(
                new ResetElevator(),                
                new EjectAutomationAuto(ElevatorConstance.highPoseCone + 0.03),
                                swerve.getAutonomousPathCommand("center to climb 1", true , 1 , 1),
                                swerve.getAutonomousPathCommand("center to climb 2", false , 3.8 , 2),
                                new WaitCommand(0.6),
                                new ParallelDeadlineGroup(
                                                new WaitUntilCommand(ClimeCommunity::isAtClimbAngle),
                                                swerve.getAutonomousPathCommand("center to climb 3", false , 3 , 2)),
                                new WaitCommand(SwerveConstants.TIME_TO_CLIMB).raceWith(
                                                new AutoBalance()),
                                new ParallelDeadlineGroup(
                                                new WaitCommand(0.2),
                                                new LockModules()),
                                new InstantCommand(() -> Leds.getInstance().setAutostate(Autostate.BALAMCED)));

        }
}
