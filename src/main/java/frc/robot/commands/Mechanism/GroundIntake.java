// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Mechanism;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

public class GroundIntake extends Command {

  private Timer timer;
  private double time = 0;


  /** Creates a new GroundIntakeWithSensor. */
  public GroundIntake(double timeout) {
     addRequirements(RobotContainer.cassetteintake, RobotContainer.cassetteangle);
    timer = new Timer();
    time=timeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
    timer.reset();
    timer.start();
    RobotContainer.cassetteangle.setAngle(CassetteEffector.GROUND_ANGLE);

    RobotContainer.cassetteintake.intakeRun(0.78);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);
    RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (timer.hasElapsed(time)|| RobotContainer.cassetteintake.NoteOrNoNote()==true)
    return true;
    else return false;
  }
}
