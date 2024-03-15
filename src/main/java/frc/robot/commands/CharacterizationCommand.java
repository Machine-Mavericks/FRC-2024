// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

public class CharacterizationCommand extends Command {
  /** Creates a new CharacterizationCommand. */
  public CharacterizationCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteangle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cassetteangle.setAngle(RobotContainer.operatorinterface.EffectorTarget.getDouble(0.05));
    RobotContainer.cassetteshooter.leftShootRun(RobotContainer.operatorinterface.ShotCharacterizationTargetL.getDouble(0));
    RobotContainer.cassetteshooter.rightShootRun(RobotContainer.operatorinterface.ShotCharacterizationTargetR.getDouble(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Quick and dirty
    if (RobotContainer.cassetteshooter.isShooterAtSpeedSetpoint() && RobotContainer.cassetteangle.isEffectorAtTarget()) {
      RobotContainer.cassetteintake.intakeRun(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE);
    RobotContainer.cassetteshooter.stopShooter();
    RobotContainer.cassetteintake.intakeRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
