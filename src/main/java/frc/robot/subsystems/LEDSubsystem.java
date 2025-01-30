// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final AddressableLED ledStrip = new AddressableLED(0);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(10);
  
  public LEDSubsystem() {
    // four score and seven years ago.......
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public void redLED() {
    // basically this makes it red ahaha
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void redorangeLED() {
    // basically this makes it yelgren ahaha
    LEDPattern yelgren = LEDPattern.solid(Color.kOrangeRed);
    yelgren.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void orangeLED() {
    // basically this makes it orange ahaha
    LEDPattern orange = LEDPattern.solid(Color.kOrange);
    orange.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void yellowLED() {
    // basically this makes it yellow ahaha
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    yellow.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
  
  public void greenyellowLED() {
    // basically this makes it yelgren ahaha
    LEDPattern grenyell = LEDPattern.solid(Color.kGreenYellow);
    grenyell.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void greenLED() {
    // basically this makes it green ahaha
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    green.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }  
  
  public void blueLED() {
    // basically this makes it blueh ahaha
    LEDPattern blueh = LEDPattern.solid(Color.kBlue);
    blueh.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }  

  public void denimLED() {
    // basically this makes it demim ahaha
    LEDPattern demim = LEDPattern.solid(Color.kDenim);
    demim.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }  

  public void bluevioletLED() {
    // basically this makes it bluviol ahaha
    LEDPattern bluviol = LEDPattern.solid(Color.kBlueViolet);
    bluviol.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  } 

  public void rainbowLED () {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    Distance kLedSpacing = Meters.of(1 / 120.0);
    LEDPattern scrollingRainbow =
      rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
    scrollingRainbow.applyTo(ledBuffer);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
