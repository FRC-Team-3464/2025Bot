// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  // Assigning variables to the motors and other objects 
    private final SparkMax leftElevator = new SparkMax(9, MotorType.kBrushless); 
    private final SparkMax rightElevator = new SparkMax(10, MotorType.kBrushless);

    private final DutyCycleEncoder absElevatorEncoder = new DutyCycleEncoder(2);
    private final RelativeEncoder leftEncoder = leftElevator.getEncoder();
    private final RelativeEncoder rightEncoder = rightElevator.getEncoder();

    // assigning the limit switches
    private final DigitalInput minElevatorLimit = new DigitalInput(0);
    private final DigitalInput maxElevatorLimit = new DigitalInput(1);
    
    // configures the right motor to be newly configured 
    private SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();


    Boolean LeftResistance;
    Boolean RightResistance;


  public ElevatorSubsystem() {
    // setting resistance/directions both motors will run in  
    LeftResistance = false;
    RightResistance = false; 
    // configuring the right sparkmax to follow the left elevator motor
    rightSparkMaxConfig.follow(leftElevator, true);
    // sets the right elevator motor to actually be configured to the code that sets the configuration
    rightElevator.configure(rightSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // CREATING GET INSTANCE
  private static ElevatorSubsystem instance = null;
  
  public static ElevatorSubsystem getInstance(){
    if (instance == null) {
      instance= new ElevatorSubsystem();  
    }
    return instance;
  }
  
  public void setElevateTarget(double target) {
    // checking which direction the elevator has to travel in to reach the target parameter after accounting for limit switches
        System.out.println("setElevateTarget should be running");
        System.out.println(Math.abs(Constants.ElevatorConstants.kElevatorGearRatio * (target) - getRelativeElevatorPosition()));

    if (getMinElevatorLimit()) {
      leftElevator.set(0);
      System.out.println("speed set to 0 because bottom limit switch");
    }
    else if (getMaxElevatorLimit()) {
      leftElevator.set(0);
      System.out.println("speed set to 0 because top limit switch");
    }
    else if (Math.abs(Constants.ElevatorConstants.kElevatorGearRatio * (target) - getRelativeElevatorPosition()) < 5) {
      leftElevator.set(0);
    }
    else if (Math.abs(Constants.ElevatorConstants.kElevatorGearRatio * (target) - getRelativeElevatorPosition()) > 5 && getElevatorHeight() < target) {
      leftElevator.set(0.6);
      System.out.println("speed set to 0.6");
    }
    else if (Math.abs(Constants.ElevatorConstants.kElevatorGearRatio * (target) - getRelativeElevatorPosition()) > 5 && getElevatorHeight() > target) {
      leftElevator.set(-0.6);
      System.out.println("speed set to -0.6");
    }
  }

  // setting the default speed of the left elevator at zero while also accounting for limit switches
  public void runElevator(double speed) {
    if (getMinElevatorLimit()) {
      leftElevator.set(0);
    }
    else if (getMaxElevatorLimit()) {
      leftElevator.set(0);
    }
    else {
      leftElevator.set(speed);
    }
  }

  public double getRelativeElevatorPosition() {
    // gets number of rotations of left relative encoder
    return leftEncoder.getPosition();
  }

  public double getElevatorPosition() {
    // gets elevator position based on number of rotations of duty cycle encoder
    return absElevatorEncoder.get();
  }

  public double getElevatorHeight() {
    // gets elevator height in inches based on number of rotations of the duty cycle encoder
    double rotations = getRelativeElevatorPosition();
    return (rotations/Constants.ElevatorConstants.kElevatorGearRatio);
    // change the kElevatorGearRatio later to get from CAD
  }

  public boolean getMaxElevatorLimit() {
    // checks if maximum limit switch is hit
    return !maxElevatorLimit.get();
  }
  public boolean getMinElevatorLimit() {
    // checks if minimum limit switch is hit
    return !minElevatorLimit.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Min Elevator Limit", getMinElevatorLimit());
    SmartDashboard.putBoolean("Max Elevator Limit", getMaxElevatorLimit());
    // This method will be called once per scheduler run
  }

  
}
