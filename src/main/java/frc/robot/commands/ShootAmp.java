// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * TODO: figure out if amp shot will be forward or backward through cassette
 */
public class ShootAmp extends Command {
  /** Creates a new ShootAmp. */
  public ShootAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassettemotor); // add effector
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int lpercent=60;
    int rpercent=100;
    // set to amp shoot angle
    // TODO: figure out if these (v) need to be swapped
    RobotContainer.cassettemotor.leftShootRun(lpercent);
    RobotContainer.cassettemotor.rightShootRun(rpercent);
    while (!RobotContainer.cassettemotor.shooterAtSpeed(lpercent, rpercent)){};
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
