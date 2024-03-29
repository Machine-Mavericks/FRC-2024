// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.util.AutoFunctions;

public class PreemptiveCassetteAngleCommand extends Command {
  private final CassetteEffector effector;

  /** Creates a new PreemptiveCassetteAngleCommand. */
  public PreemptiveCassetteAngleCommand(CassetteEffector cassetteEffector) {
    effector = cassetteEffector;
    addRequirements(effector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driverWallPosition = DriverStation.getAlliance().get() == Alliance.Red ? AutoFunctions.FIELD_X_SIZE : 0;

    // If over center line, set effector angle
    if ((Math.abs(RobotContainer.odometry.getPose2d().getX() - driverWallPosition)) < 6) {
      effector.setAngle(RobotContainer.speakertargeting.getDesiredAngle());
    } else {
      effector.setAngle(CassetteEffector.NEUTRAL_ANGLE);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
