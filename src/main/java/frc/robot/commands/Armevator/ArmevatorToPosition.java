// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Armevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmevatorToPosition extends Command {
  /** Creates a new ArmevatorToPosition. */
  private final ArmSubsystem armSub;
  private final ElevatorSubsystem elevatorSub;
  private int target;

  public ArmevatorToPosition(int position) {
    armSub = ArmSubsystem.getInstance();
    elevatorSub = ElevatorSubsystem.getInstance();

    addRequirements(armSub);
    addRequirements(elevatorSub);

    target = position;

    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (elevatorSub.getElevatorPosition() < ArmevatorConstants.kElevatorDangerPosition && armSub.getRelativeArmPosition() < ArmevatorConstants.kArmDangerPosition) {
    //   armSub.setArmTarget(ArmevatorConstants.positions[target][2]);
    // }
    // else {
      elevatorSub.setElevatorTarget(ArmevatorConstants.positions[target][1]);
      armSub.setArmTarget(ArmevatorConstants.positions[target][2]);
    // }

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
