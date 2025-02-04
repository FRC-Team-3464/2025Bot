// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunPivoterCommand extends Command {
  private ArmSubsystem armSub;
  public boolean direction;
  /** Creates a new RunPivoterCommand. */
  public RunPivoterCommand(boolean direction) {
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
    if (direction == true) {
      armSub.runPivoter(0.5);
    }
    else if (direction == false) {
      armSub.runPivoter(-0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.runPivoter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
