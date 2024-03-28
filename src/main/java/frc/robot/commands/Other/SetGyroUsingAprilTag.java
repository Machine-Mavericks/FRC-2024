// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Other;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetGyroUsingAprilTag extends Command {
  /** Creates a new SetGyroUsingAprilTag. */
  public SetGyroUsingAprilTag() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.gyro.resetGyro();
    RobotContainer.gyro.setGyro(RobotContainer.odometry.getPose2d().getRotation().getDegrees()+180.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}