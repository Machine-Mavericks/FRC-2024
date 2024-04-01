// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PreemptiveSpinUpShot extends Command {
  /** Creates a new PreemptiveSpinUpShot. */
  public PreemptiveSpinUpShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteshooter);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d pose =RobotContainer.odometry.getPose2d();

    if (RobotContainer.speakertargeting.getSpeakerDistance(pose)<= 6 && RobotContainer.cassetteintake.NoteOrNoNote()){
    RobotContainer.cassetteshooter.leftShootRun(RobotContainer.speakertargeting.getDesiredLSpeed());
    RobotContainer.cassetteshooter.rightShootRun(RobotContainer.speakertargeting.getDesiredRSpeed());
    } 
    else{
      RobotContainer.cassetteshooter.stopShooter();
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
