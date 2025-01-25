// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  public static SwerveSubsystem instance;
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] swerveMods;
  public AHRS gyro;
  
  public SwerveSubsystem() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.zeroYaw();

    swerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.ModConstants.Mod0.constants),
      new SwerveModule(1, Constants.ModConstants.Mod1.constants),
      new SwerveModule(2, Constants.ModConstants.Mod2.constants),
      new SwerveModule(3, Constants.ModConstants.Mod3.constants)
    };

    Timer.delay(1);
    resetModulesToAbsolute();

    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getModulePositions());

  }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance = new SwerveSubsystem();
    }
    return instance;
  }
 
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for(SwerveModule mod : swerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
}    

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : swerveMods) {
        positions[mod.getModuleNumber()] = mod.getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(SwerveModule mod : swerveMods){
        states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry() {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), getPose());
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
    
    for(SwerveModule mod : swerveMods){
        mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }    

  public void driveRobotRelative(ChassisSpeeds speeds){
    SwerveModuleState[] states = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveConstants.maxSpeed);
    setModuleStates(states);
  }

  public void resetModulesToAbsolute(){
    for(SwerveModule mod : swerveMods){
        mod.resetToAbsolute();
    }
  }

  public Rotation2d getYaw() {
    return gyro.getRotation2d();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
