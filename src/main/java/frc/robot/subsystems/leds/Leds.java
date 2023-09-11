// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;



import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.PortMap.Intake;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  private AddressableLED L_led;
  private AddressableLEDBuffer L_ledBuffer;
  private AddressableLED R_led;
  private AddressableLEDBuffer R_ledBuffer;
  private int firstHue = 0;
  private static Leds leds;
  private boolean on;
  private double lastChange;
  private Color[] colors;
  private Color[] color;
  private String gamePice;

  public Leds() {
    L_led = new AddressableLED(PortMap.Led.ledPort);
    L_ledBuffer = new AddressableLEDBuffer(80);
    L_led.setLength(L_ledBuffer.getLength());
    L_led.setData(L_ledBuffer);
    L_led.start();

  

  }

  public void setColorRGB(int Red , int Green , int Blue) {
    
    for (var i = 0; i < L_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      L_ledBuffer.setRGB(i, Red, Green, Blue);
      //R_ledBuffer.setRGB(i, Red, Green, Blue);
   }
   
   L_led.setData(L_ledBuffer);
   //R_led.setData(L_ledBuffer);
  }

  public void setColor(Color color) {
    
    for (var i = 0; i < L_ledBuffer.getLength(); i++) {
      
      L_ledBuffer.setLED(i, color);
   }
   L_led.setData(L_ledBuffer);
  }

  public void rainbow() {
    int currentHue;
    int length = L_ledBuffer.getLength();
    for (int i = 0; i < length; i++) {
        currentHue = (firstHue + (i * 180 / length)) % 180;
        L_ledBuffer.setHSV(i, currentHue, 255, 128);
    }

    firstHue = (firstHue + 3) % 180;
  }

  public void blink( double interval ,Color color1, Color color2) {
    double timestamp = Timer.getFPGATimestamp();
    if (timestamp - lastChange > interval) {
        on = !on;
        lastChange = timestamp;
    }
    if (on) {
        setColor(color1);;
    }
    else {
        setColor(color2);
    }
  }

  public void Wave(int period , int numColors , Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp() % period;
    double progress = elapsedTime / period;
    int numLeds = L_ledBuffer.getLength();

    for (int i = 0; i < numLeds; i++) {
        double position = (double)i / (double)numLeds;
        double wavePosition = (position + progress) % 1.0;
        int colorIndex = (int)(wavePosition * numColors);
        Color currentColor = colors[colorIndex];
        L_ledBuffer.setLED(i, currentColor);

    }
  }

  public void SmoothWave(int numColors, double period, double speed, Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp();
    for (int i = 0; i < L_ledBuffer.getLength(); i++) {
        double position = ((double) i / L_ledBuffer.getLength()) + (elapsedTime * speed / period);
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

        L_ledBuffer.setLED(i, currentColor);
    }
}

  public void setAllianceColor() {
    if (DriverStation.isFMSAttached()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        // setSolidColor(Color.kRed);
        SmoothWave(2, 1, 1, new Color [] {LedsConstants.RED, LedsConstants.BLACK});
      } else if (DriverStation.getAlliance() == Alliance.Blue) {
        // setSolidColor(Color.kBlue);
        SmoothWave(2, 1, 1, new Color [] {LedsConstants.BLUE, LedsConstants.BLACK});
      }
    }
    else{
      SmoothWave(2, 1, 1, new Color [] {LedsConstants.CUBE_PURPLE,LedsConstants.BLACK});
      // setSolidColor(Color.kPurple);
    }
  }

  public void setGamepice(String gamePice){
    this.gamePice = gamePice;
  }

  public void updateLeds() {
    L_led.setData(L_ledBuffer);

  }

  public static Leds getInstance() {
    if (leds == null) {
        leds = new Leds();  
    }
    return leds;
  }

  @Override
  public void periodic() {
    

    /* 
    if (gamePice == "cone"){
      Leds.getInstance().blink(0.6, LedsConstants.CUBE_PURPLE , LedsConstants.BLACK);
      Leds.getInstance().updateLeds();
    }else if (gamePice == "cube"){
      Leds.getInstance().blink(0.6, LedsConstants.CONE_YELLOW , LedsConstants.BLACK);
      Leds.getInstance().updateLeds();
    }/*else if (frc.robot.subsystems.intake.Intake.getInstance().isPieceInIntake() == false ) {
      Leds.getInstance().SmoothWave(2, 0.2, 0.6, new Color [] {LedsConstants.WHITE , LedsConstants.MAcolor});
      Leds.getInstance().updateLeds();
    }*/
    
  }
  
  
  
  
}