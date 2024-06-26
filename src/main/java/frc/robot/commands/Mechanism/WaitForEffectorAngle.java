// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WaitForEffectorAngle extends Command {
  /** Maximum time shooter will try to hit target speed */
  private static final double ANGLE_TIMEOUT = 1.0;
  private Timer timer;

  /** True if shot hasn't timed out */
  private boolean validShot = true;

  /** Creates a new WaitForEffectorAngle. */
  public WaitForEffectorAngle() {
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted && validShot) {
      System.out.println("Hit effector angle");
      RobotContainer.operatorinterface.ShooterAtAngle.setBoolean(true);
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(ANGLE_TIMEOUT)) {
      System.out.println("Error: Shooter failed to hit target angle");
      validShot = false;
      return true;
    }
    return RobotContainer.cassetteangle.isEffectorAtTarget();
  }
}
