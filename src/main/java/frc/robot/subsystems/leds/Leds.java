// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.LedsConstants;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  private AddressableLED L_led;
  private AddressableLEDBuffer L_ledBuffer;
  private int firstHue = 0;
  private static Leds leds;
  private boolean on;
  private double lastChange;
  private Color[] colors;
  private Color[] color;


  public Leds() {
    L_led = new AddressableLED(PortMap.Led.ledPort);
    L_ledBuffer = new AddressableLEDBuffer(76);
    L_led.setLength(L_ledBuffer.getLength());
    L_led.setData(L_ledBuffer);
    L_led.start();

  }

  public void setColorRGB(int Red , int Green , int Blue) {
    
    for (var i = 0; i < L_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      L_ledBuffer.setRGB(i, Red, Green, Blue);
   }
   
   L_led.setData(L_ledBuffer);
  }

  public void setColor(Color color) {
    
    for (var i = 0; i < L_ledBuffer.getLength(); i++) {
      
      L_ledBuffer.setLED(i, color);
   }
   
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

  public void updateLeds() {
    L_led.setData(L_ledBuffer);
  }

  public void WavePattern(int period , int numColors , Color[] colors) {
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

  public static Leds getInstance() {
    if (leds == null) {
        leds = new Leds();  
    }
    return leds;
  }

  @Override
  public void periodic() {
    
    
    Leds.getInstance().blink(1, LedsConstants.MAcolor, LedsConstants.WHITE);
    Leds.getInstance().updateLeds();
  
  
  }
  
}
