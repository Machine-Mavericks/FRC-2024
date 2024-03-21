// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Odometry;
import frc.robot.util.AprilTagMap;

public class SpinupSpeaker extends Command {
  /** Creates a new AimThenShootSpeaker. */
  public SpinupSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cassetteangle.setAngle(RobotContainer.speakertargeting.getDesiredAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Calculate our distance to target
    // find current position
    Pose2d currentPose=RobotContainer.odometry.getPose2d();

    double rangetotarget =0.0;

    // find speaker position
    Pose2d speakerPose;
    if (DriverStation.getAlliance().get() == Alliance.Red){
      speakerPose=AprilTagMap.AprilTags[3];
      // find differences in position
      double xDif = speakerPose.getX()-currentPose.getX();
      double yDif = speakerPose.getY()-currentPose.getY();
      rangetotarget = Math.sqrt(xDif*xDif + yDif*yDif);
     
    } else {
      speakerPose=AprilTagMap.AprilTags[6];
      // find differences in position
      double xDif = speakerPose.getX()-currentPose.getX();
      double yDif = speakerPose.getY()-currentPose.getY();
      rangetotarget = Math.sqrt(xDif*xDif + yDif*yDif);
    }


   
    // Contiunally adjust
    //if (RobotContainer.speakertargeting.IsTarget()) {
      RobotContainer.cassetteangle.setAngle(RobotContainer.speakertargeting.getDesiredAngle(rangetotarget));
    //}
    RobotContainer.cassetteshooter.leftShootRun(RobotContainer.speakertargeting.getDesiredLSpeed());
    RobotContainer.cassetteshooter.rightShootRun(RobotContainer.speakertargeting.getDesiredRSpeed());
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
