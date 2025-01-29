// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeSubsystem. */
  private final SparkMax algaePivotMotor = new SparkMax(14, MotorType.kBrushless);
  private final SparkMax algaeMotor = new SparkMax(15, MotorType.kBrushless);
  private final SparkMax invertedAlgaeMotor = new SparkMax(16, MotorType.kBrushless);
  private final RelativeEncoder algaePivotEncoder = algaePivotMotor.getEncoder();
  private final RelativeEncoder algaeEncoder = algaeMotor.getEncoder();
  private SparkMaxConfig algaePivotConfig;

  private final DigitalInput stowLimit = new DigitalInput(7);
  private final DigitalInput algaeMaxLimit = new DigitalInput(8);
  private final DigitalInput algaeSensor = new DigitalInput(9);
  
  public static AlgaeSubsystem instance = new AlgaeSubsystem();
  
  public AlgaeSubsystem() {
    algaePivotConfig.follow(14, true);
    invertedAlgaeMotor.configure(algaePivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public static AlgaeSubsystem getInstance() {
    if (instance == null) {
      instance = new AlgaeSubsystem();
    }
    return instance;
  }

  public void DeployAlgaeIntake() {
    if(algaeMaxLimit.get() == true) {
      algaePivotMotor.set(0);
    }
    else {
      algaePivotMotor.set(1);
    }
  }

  public void RetractAlgaeIntake() {
    if(stowLimit.get() == true) {
      algaePivotMotor.set(0);
    }
    else {
      algaePivotMotor.set(-1);
    }
  }

  public void runPivotMotor(double speed) {
    if (!(stowLimit.get() || (algaePivotMotor.get() < 0))) {
      algaePivotMotor.set(speed);
    }
  }

  public void runAlgaeMotor(double speed) {
    algaeMotor.set(speed);
  }

  public double getAlgaePivotPosition() {
    return algaePivotEncoder.getPosition();
  }

  public double getAlgaeIntakePosition() {
    return algaeEncoder.getPosition();
  }

  public boolean getStowLimit() {
    return stowLimit.get();
  }

  public double getAlgaeSpeed() {
    return algaeMotor.get();
  }

  public boolean getExtendedLimit() {
    return algaeMaxLimit.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per schedule per run
    SmartDashboard.putNumber("Algae Pivot Position", getAlgaePivotPosition());
    SmartDashboard.putBoolean("Algae Pivoter Stow Limit", getStowLimit());
    SmartDashboard.putBoolean("Algae Pivoter Extended Limit", getExtendedLimit());
    SmartDashboard.putNumber("Algae Motor Speed", getAlgaeSpeed());
  }
}
