// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeMoveToHoldingPosition extends Command {
  private static final double DISTANCE_SENSOR_SETPOINT = 6.3; // Measured as 4.5 but moved to 5 since noise results in issues

  /** Creates a new IntakeMoveToHoldingPosition. */
  public IntakeMoveToHoldingPosition() {
    addRequirements(RobotContainer.cassetteintake); // add effector
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cassetteintake.intakeRun(-0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.distanceSensors.getSensor1() >= DISTANCE_SENSOR_SETPOINT;
  }
}
