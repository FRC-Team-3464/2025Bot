// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Armevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunArm extends Command {
  private ArmSubsystem armSub;
  public boolean direction;
  /** Creates a new RunPivoterCommand. */
  public RunArm(boolean direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSub = ArmSubsystem.getInstance();
    this.direction = direction;
    addRequirements(armSub);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction) {
      armSub.runArm(0.3);
    }
    else if (!direction) {
      armSub.runArm(-0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.runArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
