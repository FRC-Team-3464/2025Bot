// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public final PhotonCamera frontAprilCamera;
  public static VisionSubsystem instance;

  public VisionSubsystem() {
    frontAprilCamera = new PhotonCamera("frontAprilCamera");
  }

  public static VisionSubsystem getInstance() {
    if (instance == null) {
      instance = new VisionSubsystem();
    }
    return instance;
  }

  public PhotonCamera getFrontCamera() {
    return frontAprilCamera;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
