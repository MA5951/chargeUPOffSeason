package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrivetrainSubsystem;

public class AutoBalance extends CommandBase {

  private final SwerveDrivetrainSubsystem swerve;
  private final PIDController pid;
  private double timeAtSetpoint;
  private boolean wasAtSetpoint;

  public AutoBalance(SwerveDrivetrainSubsystem swerveSubsystem) {
    swerve = swerveSubsystem;
    addRequirements(swerve);

    pid = new PIDController(
        SwerveConstants.AUTO_BALANCE_KP,
        SwerveConstants.AUTO_BALANCE_KI,
        SwerveConstants.AUTO_BALANCE_KD
    );
    pid.setTolerance(SwerveConstants.BALANCE_POSITION_TOLERANCE);
  }

  @Override
  public void initialize() {
    pid.setSetpoint(SwerveConstants.BALANCE_SETPOINT);
    timeAtSetpoint = 0.0;
    wasAtSetpoint = false;
  }

  @Override
  public void execute() {
    // Calculate the output of the PID controller based on the pitch
    double output = pid.calculate(swerve.getPitch());
    swerve.drive(output, 0, 0, false);

    // Check if the PID controller reached the setpoint
    if (pid.atSetpoint() && !wasAtSetpoint) {
      timeAtSetpoint = getTime();
      wasAtSetpoint = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drive when the command ends
    swerve.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Finish the command if the setpoint was reached and the delay has passed
    return wasAtSetpoint && getTime() - timeAtSetpoint > SwerveConstants.BALANCE_DELAY;
  }

  private double getTime() {
    return System.currentTimeMillis() / 1000.0;
  }
}
