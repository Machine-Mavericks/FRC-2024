// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Other;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;

public class ManualOdometryReset extends Command {
  /** Creates a new ManualOdometryReset. */
  public ManualOdometryReset() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (OI.driverController.getRightTriggerAxis() > 0.3)
    {
      if (DriverStation.getAlliance().get()==Alliance.Blue)
      {
        RobotContainer.gyro.setGyro(180.0); 
        RobotContainer.odometry.setPosition(1.3, 5.55, 180.0, 180.0);
      }
      else
      {
        RobotContainer.gyro.setGyro(0.0); 
        RobotContainer.odometry.setPosition(1.3, 16.579-5.55, 0.0, 0.0);
      }
      
    }

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
