// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoSpinUpShot extends Command {
  /** Creates a new AutoSpinUpShot. */
  public AutoSpinUpShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteshooter);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.speakertargeting.getSpeakerDistance()<= 6 && RobotContainer.cassetteintake.NoteOrNoNote()){
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
