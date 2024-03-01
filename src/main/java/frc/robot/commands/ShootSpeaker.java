// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootSpeaker extends Command {

  int lpercent;
  int rpercent;

  boolean shooterHitSpeed = false;

  /** Creates a new ShootSpeaker. */
  public ShootSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteshooter); // add effector
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cassetteshooter.leftShootRun(100);
    RobotContainer.cassetteshooter.rightShootRun(100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!shooterHitSpeed){
    //   if (RobotContainer.cassettemotor.shooterAtSpeed(lpercent, rpercent)) {
    //     shooterHitSpeed = true;
    //   }
    
  //   RobotContainer.cassetteintake.intakeRun(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // RobotContainer.cassetteintake.intakeRun(0);
    // new DelayCommand(1); // TODO: figure out how long it takes the note to be shot after photosensor does not see it
    // RobotContainer.cassettemotor.stopShooter();
    // set angle to neutral
    RobotContainer.cassetteshooter.leftShootRun(0);
    RobotContainer.cassetteshooter.rightShootRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !RobotContainer.cassettemotor.getSwitch();
    return false;
  }
}
