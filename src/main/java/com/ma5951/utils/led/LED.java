// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.led;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;



public class LED extends SubsystemBase {


  /** Creates a new LEDSubsystem. */
  private static LED led;
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

  Color gamePieceColor = LedConstance.WHITE;

  public LED() {
    ledController = new AddressableLEDController(PortMap.Led.ledPort, 150);
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

  public void setgamePiece(Color gamePieceColor){
    this.gamePieceColor = gamePieceColor;
  }

  public void setAllianceColor() {
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        // setSolidColor(Color.kRed);
        setSmoothWave(2, 1, 1, new Color [] {LedConstance.RED, LedConstance.BLACK});
      } else if (DriverStation.getAlliance() == Alliance.Blue) {
        // setSolidColor(Color.kBlue);
        setSmoothWave(2, 1, 1, new Color [] {LedConstance.BLUE, LedConstance.BLACK});
      }
    }
    else{
      setSmoothWave(2, 1, 1, new Color [] {LedConstance.CUBE_PURPLE,LedConstance.BLACK});
      // setSolidColor(Color.kPurple);
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
    if (DriverStation.isDisabled()) {
      setAllianceColor();
    }
    else if (DriverStation.isEStopped()){
      setWaveBlink(LedConstance.RED, LedConstance.WHITE, 2);
    }
    else if (DriverStation.isAutonomous()) {
      setSmoothWave(3, 1, 1, new Color [] {LedConstance.CONE_YELLOW, LedConstance.CUBE_PURPLE, LedConstance.CYAN});
    }
    else if (DriverStation.isTeleop() && !DriverStation.isJoystickConnected(0)){
      setBlinking(LedConstance.RED, LedConstance.WHITE, 0.5);
    }
    else if(gamePieceColor == LedConstance.CONE_YELLOW || gamePieceColor == LedConstance.CUBE_PURPLE){
      setSolidColor(gamePieceColor);
    }
  }
}