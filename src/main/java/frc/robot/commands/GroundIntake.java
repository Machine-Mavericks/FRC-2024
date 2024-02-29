// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class GroundIntake extends Command {
  boolean inorout;

  /** Creates a new GroundIntake. */
  public GroundIntake(boolean inorout) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteintake); // add effector
    this.inorout=inorout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set angle to ground
    if (inorout){
      RobotContainer.cassetteintake.intakeRun(1);
    } else {
      RobotContainer.cassetteintake.intakeRun(-1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);
    //set angle to neutral
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // RobotContainer.cassettemotor.getSwitch();
  }
}
