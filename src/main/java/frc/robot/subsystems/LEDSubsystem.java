// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public static LEDSubsystem instance;
  private final AddressableLED ledStrip = new AddressableLED(1);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(10);

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }
  
  public LEDSubsystem() {
    // four score and seven years ago.......
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public void setOff() {
    LEDPattern off = LEDPattern.kOff;
    off.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void redLED() {
    // basically this makes it red ahaha
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void redorangeLED() {
    // basically this makes it rorange ahaha
    LEDPattern rorange = LEDPattern.solid(Color.kOrangeRed);
    rorange.applyTo(ledBuffer);
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
    // basically this makes it grenyell ahaha
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
    // basically this makes it all of the above ahaha - micromax, 2025, also 2025 ahahah
    // guys guys guys look raibowee!!!! - Juliet Caufield, 2025
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern scrollingRainbow =
      rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpacing);
    scrollingRainbow.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
 
  public void blugrenConGradientLED () {
    // four score and fifty TRILLION years agoo... - aberhum something, 2024
    // no but really all this does is make a gradient between gren and blu - ben frankin, 3024
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlue);
    gradient.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void yellredConGradientLED () {
    //oh my god... NOW THE'RE YELLOW AND RED?!?!?!?!?? SUCH INNOVATION!!!! 
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kRed);
    gradient.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
  
  public void redbluNonContGradLED () {
    // for when you don't want a continuous gradient... obviously
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
    gradient.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  //manhunt vs 4 GOVERNMENT AGENTS IRL!!
  // trust me bro it's legal <3

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, whatever that means
  
  }
}
