// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

// DECLARE NECESSARY OBJECTS
private final SparkMax leftClimbMotor = new SparkMax(17, MotorType.kBrushless);
private final SparkMax rightClimbMotor = new SparkMax(18, MotorType.kBrushless);

// LIMIT SWITCHES
private final DigitalInput climberLimit = new DigitalInput(9);

// SPARKMAX / MOTOR CONFIG
  private SparkMaxConfig leftMotorConfig;

  /** Creates a new ClimberSubsystem. */
  // Left motors FOLLOWS right motor and is INVERTED
  public ClimberSubsystem() {
leftMotorConfig.follow(18, true);
leftClimbMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

// look at the method name what do you think it does
public boolean getClimberLimit() {
  return climberLimit.get();
}

// runs rightClimbMotor, leftClimbMotor is following already
public void runClimbMotors(double speed) {
  rightClimbMotor.set(speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
