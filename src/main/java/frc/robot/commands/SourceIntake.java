// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SourceIntake extends Command {
  /** Creates a new SourceIntake. */
  public SourceIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassettemotor); // add effector
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set angle to source
    RobotContainer.cassettemotor.leftShootRun(-100);
    RobotContainer.cassettemotor.rightShootRun(-100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassettemotor.stopShooter();
    //set angle to neutral
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.cassettemotor.getSwitch();
  }
}
