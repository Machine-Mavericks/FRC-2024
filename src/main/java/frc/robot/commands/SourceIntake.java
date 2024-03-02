// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

public class SourceIntake extends Command {
  
  private Timer intakeTimer;

  
  /** Creates a new SourceIntake. */
  public SourceIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteintake, RobotContainer.cassetteangle); // add effector
    intakeTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeTimer.reset();
    intakeTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.cassetteangle.setAngle(CassetteEffector.SOURCE_ANGLE);
    new DelayCommand(1);
    RobotContainer.cassetteintake.intakeRun(1);
    RobotContainer.cassetteintake.intakeRun(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);

    RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE);
    //set angle to neutral
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeTimer.hasElapsed(3);
  }
}
