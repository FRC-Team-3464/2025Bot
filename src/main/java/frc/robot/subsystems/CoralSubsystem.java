// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new CoralSubsystem. */
  public static CoralSubsystem instance;
  private final SparkMax coralMotor = new SparkMax(14, MotorType.kBrushless);
  private final RelativeEncoder coralEncoder = coralMotor.getEncoder();
  private final DigitalInput coralPhotoElectric = new DigitalInput(6); 

  public CoralSubsystem() {}

 public void runIntake(double speed) {
    coralMotor.set(speed);
  }

  public void stopIntake() {
    coralMotor.set(0);
  }

  public boolean getPhotoElectric() {
    return coralPhotoElectric.get();
  }

  public static CoralSubsystem getInstance() {
    if (instance == null) {
      instance = new CoralSubsystem();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 

}