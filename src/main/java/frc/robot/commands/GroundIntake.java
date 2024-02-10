// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class GroundIntake extends Command {
  /** Creates a new GroundIntake. */
  public GroundIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassettemotor); // add effector
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set angle to ground
    RobotContainer.cassettemotor.intakeRun(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassettemotor.intakeRun(false);
    //set angle to neutral
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.cassettemotor.getSwitch();
  }
}
