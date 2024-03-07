// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.crypto.KeySelector.Purpose;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

public class GroundIntake extends Command {
  private static Command MoveToHoldingPositionCommand = new IntakeMoveToHoldingPosition();
  private Timer timer;

  private boolean inorout;
  private boolean useTimeout = false;
  private double timeout = 0;

  /** Creates a new GroundIntake. */
  public GroundIntake(boolean inorout) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteintake, RobotContainer.cassetteangle); // add effector
    timer = new Timer();

    this.inorout=inorout;
  }

  /** Creates a new GroundIntake. */
  public GroundIntake(boolean inorout, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteintake, RobotContainer.cassetteangle); // add effector
    timer = new Timer();

    this.inorout=inorout;
    useTimeout = true;
    this.timeout = timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    RobotContainer.cassetteangle.setAngle(CassetteEffector.GROUND_ANGLE);

    if (inorout){
      RobotContainer.cassetteintake.intakeRun(1);
    } else {
      RobotContainer.cassetteintake.intakeRun(-1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);
    RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE);
    CommandScheduler.getInstance().schedule(MoveToHoldingPositionCommand);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (useTimeout) {
      return timer.hasElapsed(timeout);
    }
    return false;
  }
}
