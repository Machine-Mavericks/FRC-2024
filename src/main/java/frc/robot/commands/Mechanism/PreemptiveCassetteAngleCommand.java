// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.util.AutoFunctions;
import frc.robot.util.Utils;

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

    Pose2d predictedPose = AutoFunctions.estimateFuturePose(0.5, 1.5);
    RobotContainer.operatorinterface.m_field.getObject("Predicted Pose").setPose(predictedPose);
    
    if (RobotContainer.cassetteintake.NoteOrNoNote()) {
      // If closer than seven meters, set effector angle
      if ((Math.abs(predictedPose.getX() - driverWallPosition)) < 7) {
        effector.setAngle(RobotContainer.speakertargeting.getDesiredAngle());
      } else {
        effector.setAngle(CassetteEffector.NEUTRAL_ANGLE);
      }
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
