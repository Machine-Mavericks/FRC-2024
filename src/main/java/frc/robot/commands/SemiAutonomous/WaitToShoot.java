// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.AprilTagMap;

public class WaitToShoot extends Command {
  /** Creates a new WaitToShoot. */
  public WaitToShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // find speaker position
    Pose2d speakerPose;
    // find current position
    Pose2d currentPose=RobotContainer.odometry.getPose2d();
    if (DriverStation.getAlliance().get() == Alliance.Red){
      speakerPose=AprilTagMap.AprilTags[3];
    } else {
      speakerPose=AprilTagMap.AprilTags[6];
    }
    // find differences in position
    double xDif = currentPose.getX()-speakerPose.getX();
    double yDif = currentPose.getY()-speakerPose.getY();
    // find distance
    double distance = Math.sqrt(Math.pow(xDif,2)+Math.pow(yDif,2));
    boolean toofar = distance >= 6;
    return RobotContainer.cassetteintake.NoteOrNoNote()&&!toofar;
  }
}
