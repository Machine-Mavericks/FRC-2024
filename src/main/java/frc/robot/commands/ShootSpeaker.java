// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootSpeaker extends Command {

  int lpercent;
  int rpercent;

  boolean shooterHitSpeed = false;

  /** Creates a new ShootSpeaker. */
  public ShootSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassettemotor); // add effector
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // // set to speaker shoot angle
    // // TODO: figure out if these (v) need to be swapped
    // if (DriverStation.getAlliance().get() == Alliance.Blue) {
    //   lpercent=60; // TODO: switch to velocites as opposed to %output
    //   rpercent=100;
    // } else {
    //   lpercent=100;
    //   rpercent=60;
    // } 
    // RobotContainer.cassettemotor.leftShootRun(lpercent);
    // RobotContainer.cassettemotor.rightShootRun(rpercent);
    // RobotContainer.cassettemotor.leftShootRun(0.6);
    // RobotContainer.cassettemotor.rightShootRun(0.6);


    //set velocity to 8 rps, add 0.5 V to overcome gravity 
    RobotContainer.cassettemotor.leftShootRun(100);
    RobotContainer.cassettemotor.rightShootRun(100);
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
    RobotContainer.cassettemotor.leftShootRun(0);
    RobotContainer.cassettemotor.rightShootRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return !RobotContainer.cassettemotor.getSwitch();
    return false;
  }
}
