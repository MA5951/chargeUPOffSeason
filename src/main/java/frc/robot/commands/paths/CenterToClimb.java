package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Automations.ElevatorAutomations.ResetElevator;
import frc.robot.commands.ScoringAutomation.EjectAutomationAuto;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.LockModules;
import frc.robot.subsystems.elevator.ElevatorConstance;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class CenterToClimb extends SequentialCommandGroup {

        private static SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();

        private static boolean isAtClimbAngle() {
                return Math.abs(swerve.getPitch()) > SwerveConstants.ANGLE_BEFORE_CLIMB;
        }

        public CenterToClimb() {

                addCommands(
                                new ResetElevator(),
                                new EjectAutomationAuto(ElevatorConstance.highPoseCone),
                                swerve.getAutonomousPathCommand("center to climb a", true),
                                new WaitCommand(1),
                                new ParallelDeadlineGroup(
                                                new WaitUntilCommand(CenterToClimb::isAtClimbAngle),
                                                swerve.getAutonomousPathCommand("center to climb 2", false)),
                                new WaitCommand(SwerveConstants.TIME_TO_CLIMB).raceWith(
                                                new AutoBalance()),
                                new ParallelDeadlineGroup(
                                                new WaitCommand(0.2),
                                                new LockModules()));
        }
}
