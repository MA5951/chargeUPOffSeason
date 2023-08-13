package frc.robot.commands.paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.LockModules;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class CenterToClimb extends SequentialCommandGroup{
    
    private static SwerveDrivetrainSubsystem swerve = SwerveDrivetrainSubsystem.getInstance();

    private static boolean isAtClimbAngle() {
        return swerve.getPitch() > SwerveConstants.ANGLE_BEFORE_CLIMB;
    }

    public CenterToClimb() {
        
        addCommands(
            new ParallelDeadlineGroup(
                new WaitUntilCommand(CenterToClimb::isAtClimbAngle),
                swerve.getAutonomousPathCommand("from B1 to climb", false)
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(SwerveConstants.TIME_TO_CLIMB),
                new AutoBalance()
            ),
            new LockModules()
        );
    }
}
