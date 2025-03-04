// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {

  private CoralSubsystem coralSub;
  /** Creates a new IntakeCoral. */
  public IntakeCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    coralSub = CoralSubsystem.getInstance();
    addRequirements(coralSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Tune speed later, through trial & error
    coralSub.runIntake(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralSub.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralSub.getPhotoElectric();
  }
}
