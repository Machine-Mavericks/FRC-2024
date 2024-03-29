// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootSpeaker extends Command {
  private Timer timer;
  private Timer noteTimer;

  /** Creates a new ShootSpeaker. */
  public ShootSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cassetteintake, RobotContainer.cassetteshooter);
    
    timer = new Timer();
    noteTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cassetteintake.intakeRun(1);
    timer.reset();
    timer.start();
    noteTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.cassetteintake.NoteOrNoNote()){
      noteTimer.start();
    } else {
      noteTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(2.0)||noteTimer.hasElapsed(0.1);
  }
}
