// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SpinupSpeaker extends Command {

  /** Creates a new AimThenShootSpeaker. */
  public SpinupSpeaker() {
    addRequirements(RobotContainer.cassetteshooter, RobotContainer.cassetteangle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set angle based on distance
    RobotContainer.cassetteangle.setAngle(RobotContainer.speakertargeting.getDesiredAngle());

    // Get to speed
    RobotContainer.cassetteshooter.leftShootRun(RobotContainer.speakertargeting.getDesiredLSpeed());
    RobotContainer.cassetteshooter.rightShootRun(RobotContainer.speakertargeting.getDesiredRSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
