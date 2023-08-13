// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations.IntakeAutomations;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TeleopEject extends SequentialCommandGroup {
  /** Creates a new Teleop_Eject. */
  private static double getPower() {
    return Intake.getInstance().isCubeInIntake() ? IntakeConstance.EjectPowerForCube
        : IntakeConstance.EjectPowerForCone;
}

  public TeleopEject() {
    addCommands(
      new EjectAutomation(TeleopEject::getPower)
    );
  }
}
