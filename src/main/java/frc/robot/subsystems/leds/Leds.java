// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.intake.Intake;


public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  public enum GamePiece {
    CUBE,
    CONE,
    NONE
  }
  
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int firstHue = 0;
  private static Leds leds;
  private boolean on;
  private double lastChange;
  private GamePiece gamePiece;

  public Leds() {
    led = new AddressableLED(PortMap.Led.ledPort);
    ledBuffer = new AddressableLEDBuffer(LedsConstants.LED_LENGTH);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setGamePiece(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public void setSingleColorRGB(int Red , int Green , int Blue) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, Red, Green, Blue);
    }
   
    led.setData(ledBuffer);
  }

  public void setSingleColor(Color color) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }

    led.setData(ledBuffer);
  }

  public void rainbowColorPattern() {
    int currentHue;
    int length = ledBuffer.getLength();

    for (int i = 0; i < length; i++) {
        currentHue = (firstHue + (i * 180 / length)) % 180;
        ledBuffer.setHSV(i, currentHue, 255, 128);
    }

    firstHue = (firstHue + 3) % 180;
  }

  public void blinkColorPattern(double interval ,Color color1, Color color2) {
    double timestamp = Timer.getFPGATimestamp();

    if (timestamp - lastChange > interval) {
        on = !on;
        lastChange = timestamp;
    }
    if (on) {
        setSingleColor(color1);;
    }
    else {
        setSingleColor(color2);
    }
  }

  public void waveColorPattern(int period , int numColors , Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp() % period;
    double progress = elapsedTime / period;
    int numLeds = ledBuffer.getLength();

    for (int i = 0; i < numLeds; i++) {
      double position = (double) i / (double) numLeds;
      double wavePosition = (position + progress) % 1.0;
      int colorIndex = (int) (wavePosition * numColors);
      
      Color currentColor = colors[colorIndex];
      ledBuffer.setLED(i, currentColor);
    }
  }

  public void smoothWaveColorPattern(int numColors, double period, double speed, Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp();

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      double position = ((double) i / ledBuffer.getLength()) + (elapsedTime * speed / period);
      double progress = position - (int) position;

      int startColorIndex = (int) (position % numColors);
      int endColorIndex = (startColorIndex + 1) % numColors;
      Color startColor = colors[startColorIndex];
      Color endColor = colors[endColorIndex];

      Color currentColor = new Color(
              startColor.red + (endColor.red - startColor.red) * progress,
              startColor.green + (endColor.green - startColor.green) * progress,
              startColor.blue + (endColor.blue - startColor.blue) * progress
      );

      ledBuffer.setLED(i, currentColor);
    }
  }

  public void setAllianceColor() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      smoothWaveColorPattern(2, 0.2, 0.1, new Color [] {LedsConstants.RED, LedsConstants.BLACK});
      updateLeds();
    } else if (DriverStation.getAlliance() == Alliance.Blue){
      smoothWaveColorPattern(2, 0.2, 0.1, new Color [] {LedsConstants.BLUE, LedsConstants.BLACK});
      updateLeds();
    } else {
      smoothWaveColorPattern(2, 7, 0.5, new Color [] {LedsConstants.PURPLE,LedsConstants.BLACK});
      updateLeds();
    }
  }

  public void partsPattern() {
    for (var i = 0; i < 53; i++) {
      ledBuffer.setLED(i, LedsConstants.MAcolor);
    }
    led.setData(ledBuffer);
  
    for (var i = 53; i < 76; i++) { 
      ledBuffer.setLED(i, LedsConstants.WHITE);
    }
    led.setData(ledBuffer);

    for (var i = 76; i < 129; i++) {
      ledBuffer.setLED(i, LedsConstants.MAcolor);
    }
    led.setData(ledBuffer);

    for (var i = 129; i < 152; i++) {   
      ledBuffer.setLED(i, LedsConstants.WHITE);
    }
    led.setData(ledBuffer);
  }

  public void updateLeds() {
    led.setData(ledBuffer);
  }

  public static Leds getInstance() {
    if (leds == null) {
        leds = new Leds();  
    }
    return leds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      setAllianceColor();
      updateLeds();
    } else if (DriverStation.isTeleop()) {
      if (Intake.getInstance().isCubeIn() == true) {
        setSingleColor(LedsConstants.CUBE_PURPLE);
        setGamePiece(GamePiece.NONE);
        updateLeds();
      } else if (Intake.getInstance().isConeIn() == true) {
        setSingleColor(LedsConstants.CONE_YELLOW);
        setGamePiece(GamePiece.NONE);
        updateLeds();
      } else if (gamePiece == GamePiece.CONE){
        blinkColorPattern(0.5, LedsConstants.CONE_YELLOW , LedsConstants.BLACK);
        updateLeds();
      } else if (gamePiece == GamePiece.CUBE){
        blinkColorPattern(0.5, LedsConstants.CUBE_PURPLE , LedsConstants.BLACK);
        updateLeds();
      } else if (Intake.getInstance().isPieceInIntake() == false) {
        setGamePiece(GamePiece.NONE);
        partsPattern();
        updateLeds();
      }
    } else if (DriverStation.isAutonomous()) {
      smoothWaveColorPattern(3, 1, 1, new Color [] {LedsConstants.CONE_YELLOW, LedsConstants.CUBE_PURPLE, LedsConstants.CYAN});
      updateLeds();
    }
  }
}
