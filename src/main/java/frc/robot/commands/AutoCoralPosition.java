// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCoralPosition extends Command {
  /** Creates a new AutoCoralPosition. */
  public final int[] coralIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, THETA_CONSTRAINTS);
 
  public final VisionSubsystem visionSub = VisionSubsystem.getInstance();
  public final SwerveSubsystem swerveSub = SwerveSubsystem.getInstance();

  private Transform3d camToTarget;

  private final PhotonCamera photonCamera;

  public AutoCoralPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    photonCamera = visionSub.getFrontCamera();
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var cameraResults = photonCamera.getAllUnreadResults();
    if (!cameraResults.isEmpty()) {
      var result = cameraResults.get(cameraResults.size() - 1);
      if (result.hasTargets()) {
        var targetOpt = result.getTargets().stream()
          .filter(t -> Arrays.asList(coralIDs).contains(t.getFiducialId()))
          .filter(t -> t.getPoseAmbiguity() >= 0.2 && t.getPoseAmbiguity() != -1)
          .findFirst();

        if (targetOpt.isPresent()) {
          var target = targetOpt.get();

        }
      }
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
