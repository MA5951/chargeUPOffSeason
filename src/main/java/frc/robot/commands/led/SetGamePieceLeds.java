// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.LED;

public class SetGamePieceLeds extends InstantCommand {

  private LED led;
  private Color gamePieceColor;

  public SetGamePieceLeds(Color gamePieceColor) {
    led = LED.getInstance();
    this.gamePieceColor = gamePieceColor;
    addRequirements(led);

  }

  @Override
  public void initialize() {
    led.setgamePiece(this.gamePieceColor);
  }
}
