// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootSpeaker extends Command {
  /** Creates a new ShootSpeaker. */
  public ShootSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassettemotor); // add effector
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set to speaker shoot angle
    // TODO: figure out if these (v) need to be swapped
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      RobotContainer.cassettemotor.leftShootRun(60);
      RobotContainer.cassettemotor.rightShootRun(100);
    } else {
      RobotContainer.cassettemotor.leftShootRun(100);
      RobotContainer.cassettemotor.rightShootRun(60);
    } 
    new DelayCommand(1); // TODO: figure out how many seconds until shooter reaches full speed
    RobotContainer.cassettemotor.intakeRun(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassettemotor.intakeRun(false);
    new DelayCommand(1); // TODO: figure out how long it takes the note to be shot after photosensor does not see it
    RobotContainer.cassettemotor.stopShooter();
    // set angle to neutral
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotContainer.cassettemotor.getSwitch();
  }
}
