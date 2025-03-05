// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.ProfileTiming;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  
  private final SparkMax leftPivot = new SparkMax(11, MotorType.kBrushless);
  private final SparkMax rightPivot = new SparkMax(12, MotorType.kBrushless);

  private SparkClosedLoopController pivotController = leftPivot.getClosedLoopController();

  private final DutyCycleEncoder absPivoterEncoder = new DutyCycleEncoder(3);
  private static ArmSubsystem instance = null;

  private final RelativeEncoder leftEncoder = leftPivot.getEncoder();
  private final RelativeEncoder rightEncoder = rightPivot.getEncoder();

  private final DigitalInput minPivotSwitch = new DigitalInput(4);
  private final DigitalInput maxPivotSwitch = new DigitalInput(5);

  public static final SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();

  private RelativeEncoder pivotEncoder = leftPivot.getEncoder();

  static {
    leftSparkMaxConfig
      .closedLoop
      .pid(0, 0, 0, ClosedLoopSlot.kSlot0)
      .maxMotion
        .maxVelocity(2000)
        .maxAcceleration(3750)
        .allowedClosedLoopError(0.2270833333);

    leftSparkMaxConfig.smartCurrentLimit(30);
  }

  
  private SparkMaxConfig rightSparkMaxConfig;


  public ArmSubsystem() {
    leftPivot.configure(leftSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightSparkMaxConfig = new SparkMaxConfig();
    rightSparkMaxConfig.follow(11, true);
    rightPivot.configure(rightSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public void runPivoter(double speed) {
    // if (getMaxPivotLimit()) {
    //   leftPivot.set(0);
    // }
    // else if (getMinPivotLimit()) {
    //   leftPivot.set(0);
    // }
    // else {
    //   leftPivot.set(speed);
    // }
    leftPivot.set(speed);
  }

  public void pivotToPosition(double target) {
    if (getMaxPivotLimit()) {
      leftPivot.set(0);
    }
    else if (getMinPivotLimit()) {
      leftPivot.set(0);
    }
    else if (Math.abs(target - getPivoterDegrees()) < target && target < getPivoterDegrees()) {
      leftPivot.set(0.5);
    }
    else if (Math.abs(target - getPivoterDegrees()) > target && target > getPivoterDegrees()) {
      leftPivot.set(-0.5);
    }
    else {
      leftPivot.set(0);
    }
  }

  public void PIDPivotToPosition(double tatrget) {
    pivotController.setReference(tatrget, ControlType.kMAXMotionPositionControl);
  }

  public double getAbsPivotPosition() {
    return absPivoterEncoder.get();
  }

  public double getPivoterDegrees() {
    double rotations = getAbsPivotPosition();
    return rotations * (360/277);
  }

  public double getRelPivotPosition() {
    return leftEncoder.getPosition();
  }

  public boolean getMaxPivotLimit() {
    return maxPivotSwitch.get();
  }

  public boolean getMinPivotLimit() {
    return minPivotSwitch.get();
  }

  public void setEncoderPosition(double position) {
    leftEncoder.setPosition(position); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivoter Degrees", getPivoterDegrees());
    SmartDashboard.putBoolean("Pivoter Max Limit", getMaxPivotLimit());
    SmartDashboard.putBoolean("Pivoter Min Limit", getMinPivotLimit());
    // SmartDashboard.putNumber("Pivoter Setpoint", pivotToPosition());
  }
}
