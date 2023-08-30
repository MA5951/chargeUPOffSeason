// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import com.ma5951.utils.led.LED;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.led.LED2;

public class SetGamePieceLeds extends InstantCommand {

  private LED led;
  private LED2 led2;
  private Color gamePieceColor;

  public SetGamePieceLeds(Color gamePieceColor) {
    led = LED.getInstance();
    led2 = LED2.getInstance();
    this.gamePieceColor = gamePieceColor;
    addRequirements(led);
    addRequirements(led2);

  }

  @Override
  public void initialize() {
    led.setgamePiece(this.gamePieceColor);
    led2.setgamePiece(this.gamePieceColor);
  }
}
