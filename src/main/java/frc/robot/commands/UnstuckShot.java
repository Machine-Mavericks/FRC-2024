// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

public class UnstuckShot extends Command {
  private Timer intakeTimer;

  /** Creates a new UnstuckShot. */
  public UnstuckShot() {
    addRequirements(RobotContainer.cassetteintake);
    addRequirements(RobotContainer.cassetteshooter);
    addRequirements(RobotContainer.cassetteangle);
    intakeTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.reset();
    intakeTimer.start();

    RobotContainer.cassetteintake.intakeRun(-1);
    RobotContainer.cassetteshooter.leftShootRun(-30);
    RobotContainer.cassetteshooter.rightShootRun(-30);
    RobotContainer.cassetteangle.setAngle(CassetteEffector.GROUND_ANGLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);
    RobotContainer.cassetteshooter.stopShooter();
    RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
