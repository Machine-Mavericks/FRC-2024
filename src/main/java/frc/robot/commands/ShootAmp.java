// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

/**
 * TODO: figure out if amp shot will be forward or backward through cassette
 */
public class ShootAmp extends Command {

  private Timer Timer;

  /** Creates a new ShootAmp. */
  public ShootAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteshooter, RobotContainer.cassetteangle); // add effector
    Timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Timer.reset();
    Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int lpercent=60;
    int rpercent=100;
    RobotContainer.cassetteangle.setAngle(CassetteEffector.AMP_ANGLE);
    // TODO: figure out if these (v) need to be swapped
    RobotContainer.cassetteshooter.leftShootRun(500);
    RobotContainer.cassetteshooter.rightShootRun(500);
    //while (!RobotContainer.cassetteshooter.isShooterAtSpeedSetpoint()){};
    RobotContainer.cassetteintake.intakeRun(1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);
    new DelayCommand(1); // TODO: figure out how long it takes the note to be shot after photosensor does not see it
    RobotContainer.cassetteshooter.stopShooter();
    RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.hasElapsed(1);
    //TODO: No sensor implemented to detect when shot is complete, we need a solution
  }
}
