// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;



import com.ma5951.utils.led.BlinkingColorPattern;

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

  public enum Autostate {
    CLIMED,
    NONE
  }

  public enum Animation {
    BLINK_CONE,
    BLINK_CUBE,
    SOLID_CONE,
    SOLID_CUBE,
    CHARGED,
    NONE
  }
  
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int firstHue = 0;
  private static Leds leds;
  private boolean on;
  private GamePiece gamePiece;
  private Autostate autostate;
  private Animation animation;
  private double lastChange;
  private boolean lastmodecahnge;
  private boolean lastItem;


  public Leds() {
    led = new AddressableLED(PortMap.Led.ledPort);
    ledBuffer = new AddressableLEDBuffer(LedsConstants.LED_LENGTH);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }



  public void setAutostate(Autostate autostate) {
    this.autostate = autostate;
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

  public void blinkColorPattern( double interval,Color colorOne, Color colorTwo) {
    double timestamp = Timer.getFPGATimestamp();

    if (timestamp - lastChange > interval) {
        on = !on;
        lastChange = timestamp;
    }
    if (on) {
        setSingleColor(colorOne);;
    }
    else {
        setSingleColor(colorTwo);
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

  public void cahrgedPattern(Color color1 , Color color2) {
    for (var i = 0; i < 53; i++) {
      ledBuffer.setLED(i, color1);
    }
    led.setData(ledBuffer);
  
    for (var i = 53; i < 76; i++) { 
      ledBuffer.setLED(i, color2);
    }
    led.setData(ledBuffer);

    for (var i = 76; i < 129; i++) {
      ledBuffer.setLED(i, color1);
    }
    led.setData(ledBuffer);

    for (var i = 129; i < 152; i++) {   
      ledBuffer.setLED(i, color2);
    }
    led.setData(ledBuffer);
  }

  public void runAnimation(Animation animation) {
    
    if (animation == Animation.BLINK_CONE){
      blinkColorPattern(0.5 , LedsConstants.CONE_YELLOW , LedsConstants.BLACK);
      updateLeds();
    } else if (animation == Animation.BLINK_CUBE) {
      blinkColorPattern(0.5 , LedsConstants.CUBE_PURPLE , LedsConstants.BLACK);
      updateLeds();
    } else if (animation == Animation.SOLID_CONE) {
      setSingleColor(LedsConstants.CONE_YELLOW);
      updateLeds();
    } else if (animation == Animation.SOLID_CUBE) {
      setSingleColor(LedsConstants.CUBE_PURPLE);
      updateLeds();
    } else if (animation == Animation.CHARGED) {
      cahrgedPattern(LedsConstants.MAcolor, LedsConstants.WHITE);
      updateLeds();
    } else if (animation == Animation.NONE) {

    }


  }

  public void setAnimation(Animation setanimation) {
    animation = setanimation;
  }

  public void setGamePiece() {
    if (Intake.getInstance().isConeIn()) {
      animation = Animation.SOLID_CONE;
    } else if (Intake.getInstance().isCubeIn()) {
      animation = Animation.SOLID_CUBE;
    }
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
    
          
      runAnimation(animation);
      
    
  }
}

  

