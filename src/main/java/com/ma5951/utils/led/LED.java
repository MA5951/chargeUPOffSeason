// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.intake.Intake;

public class LED extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public enum GamePiece {
    CUBE,
    CONE,
    NONE
  }

  private static LED led;
  public GamePiece gamePiece;

  // import led patterns
  AddressableLEDController ledController;
  SolidColorPattern solidColorPattern;
  RainbowColorPattern rainbowColorPattern;
  RainbowColorPatterSimultaneously rainbowColorPatterSimultaneously;
  BlinkingColorPattern blinkingColorPattern;
  BreathingColorPattern breathingColorPattern;
  BreathingTripleColorPattern breathingTripleColorPattern;
  SmoothColorTransitionPattern smoothColorTransitionPattern;
  WavePattern wavePattern;
  SmoothWaveColorPattern smoothWaveColorPattern;
  WaveBlinkColorPattern waveBlinkColorPattern;
  EvenOddColorPattern evenOddColorPattern;

  public LED() {
    ledController = new AddressableLEDController(PortMap.Led.ledPort, LedConstance.LED_LENGTH);

    // constuct led patterns
    solidColorPattern = new SolidColorPattern(Color.kRed);
    rainbowColorPattern = new RainbowColorPattern();
    blinkingColorPattern = new BlinkingColorPattern(Color.kRed, Color.kRed,0);
    breathingColorPattern = new BreathingColorPattern(Color.kRed, 0);
    breathingTripleColorPattern = new BreathingTripleColorPattern(Color.kRed, Color.kBlue, 0);
    rainbowColorPatterSimultaneously = new RainbowColorPatterSimultaneously();
    smoothColorTransitionPattern = new SmoothColorTransitionPattern(Color.kRed, Color.kBlue, 0);
    wavePattern = new WavePattern(2, 5, 1, new Color [] {Color.kRed, Color.kBlue});
    smoothWaveColorPattern = new SmoothWaveColorPattern(2, 5, 1, new Color [] {Color.kRed, Color.kBlue});
    waveBlinkColorPattern = new WaveBlinkColorPattern(Color.kRed, Color.kBlue, 0);
    evenOddColorPattern = new EvenOddColorPattern(Color.kRed, Color.kBlue, 0);
  }

  public void setGamePiece(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public void setSolidColor(Color color) {
    solidColorPattern.setColor(color);
    ledController.setAddressableLEDPattern(solidColorPattern);
  }

  public void setSmoothWave(int numColors, double period, double speed, Color[] colors) {
    smoothWaveColorPattern.setParameters(numColors, period, speed, colors);
    ledController.setAddressableLEDPattern(smoothWaveColorPattern);
  }

  public void setRainbow() {
    ledController.setAddressableLEDPattern(rainbowColorPattern);
  }

  public void setSmoothColorTransition(Color color, Color color2, double interval) {
    smoothColorTransitionPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(smoothColorTransitionPattern);
  }
  
  public void setFullRainbow() {
    ledController.setAddressableLEDPattern(rainbowColorPatterSimultaneously);
  }


  public void setWave(int numColors, double period, double speed, Color[] colors) {
    wavePattern.setParameters(numColors, period, speed, colors);
    ledController.setAddressableLEDPattern(wavePattern);
  }
  
  public void setEvenOdd(Color color, Color color2, double lenght) {
    evenOddColorPattern.setParameters(color, color2, lenght);
    ledController.setAddressableLEDPattern(evenOddColorPattern);
  }


  public void setWaveBlink(Color color, Color color2, double interval) {
    waveBlinkColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(waveBlinkColorPattern);
  }


  public void setBlinking(Color color, Color color2, double interval) {
    blinkingColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(blinkingColorPattern);
  }

  public void setBreathing(Color color, double interval){
    breathingColorPattern.setParameters(color, interval);
    ledController.setAddressableLEDPattern(breathingColorPattern);
  }

  public void setBreathingTriple(Color color, Color color2, double interval){
    breathingTripleColorPattern.setParameters(color, color2, interval);
    ledController.setAddressableLEDPattern(breathingTripleColorPattern);
  }

  public void setAllianceColor() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      setSmoothWave(2, 0.2, 0.1, new Color [] {LedConstance.RED, LedConstance.BLACK});
    } else if (DriverStation.getAlliance() == Alliance.Blue){
      setSmoothWave(2, 0.2, 0.1, new Color [] {LedConstance.BLUE, LedConstance.BLACK});
    } else {
      setSmoothWave(2, 7, 0.5, new Color [] {LedConstance.PURPLE,LedConstance.BLACK});
    }
  }

  public static LED getInstance() {
    if (led == null) {
      led = new LED();
    }
    return led;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      setAllianceColor();
    } else if (DriverStation.isTeleop()) {
      setWave(2, 1, 1, new Color[] {LedConstance.MAcolor, LedConstance.WHITE});
      if (Intake.getInstance().isCubeIn() == true) {
        setSolidColor(LedConstance.CUBE_PURPLE);
        setGamePiece(GamePiece.NONE);
      } else if (Intake.getInstance().isConeIn() == true) {
        setSolidColor(LedConstance.CONE_YELLOW);
        setGamePiece(GamePiece.NONE);
      } else if (gamePiece == GamePiece.CONE){
        setBlinking(LedConstance.BLACK, LedConstance.CONE_YELLOW , 0.5);
      } else if (gamePiece == GamePiece.CUBE){
        setBlinking(LedConstance.BLACK, LedConstance.CUBE_PURPLE , 0.5);
      } else if (Intake.getInstance().isPieceInIntake() == false) {
        setGamePiece(GamePiece.NONE);
      }
    } else if (DriverStation.isAutonomous()) {
      setSmoothWave(3, 1, 1, new Color [] {LedConstance.CONE_YELLOW, LedConstance.CUBE_PURPLE, LedConstance.CYAN});
    }
  }
}