// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class OperatorSpinup extends Command {
  private static final double LEFT_RPM = 3000;
  private static final double RIGHT_RPM = 3000;

  /** Creates a new OperatorSpinup. */
  public OperatorSpinup() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteshooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set speed once
    RobotContainer.cassetteshooter.leftShootRun(LEFT_RPM);
    RobotContainer.cassetteshooter.rightShootRun(RIGHT_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // If interrupted by shoot command, don't stop shooter. Otherwise, do stop shooter
    if (!interrupted) {
      RobotContainer.cassetteshooter.stopShooter();
    }
  }

  // This is the default, so this code could be deleted wihout problems, but serves as a good reminder that interruption behaviours exist
  // It's also just good to set explicitly
  @Override
  public InterruptionBehavior getInterruptionBehavior(){
    return InterruptionBehavior.kCancelSelf; // Ensures that command will cancel itself if another command tries to use the shooter
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
