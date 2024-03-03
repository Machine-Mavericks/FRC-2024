// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AimToSpeaker extends Command {
// PID gains for rotating robot towards ball target
  double kp = 0.01;
  double ki = 0.0;
  double kd = 0.0001;
  PIDController pidController = new PIDController(kp, ki, kd);
  
  // timer counts how long robot is lined up to target for
  double OnTargetTime;
  
  /** Creates a new AimToSpeaker */
  public AimToSpeaker() {
    // aiming uses drive system
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset the PID controller
    pidController.reset();

    // reset ontarget timer to 0.0s
    OnTargetTime = 0.0;

    TargetAngle=0.0;
  }

  // angle to target
  double TargetAngle;
  int missedSamples;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if we have target, then get angle. If no target, assume 180deg
    if ((RobotContainer.speakertargeting.IsTarget()))
    {
      TargetAngle = RobotContainer.speakertargeting.getSpeakerAngle();
      missedSamples = 0;
    }
    else{
      missedSamples +=1;
      if (missedSamples >=3)
        TargetAngle = 0;
    }
      

    // calculate PID controller
    double controlleroutput = 0.0;
    // if (Math.abs(TargetAngle)>2.0 && Math.abs(TargetAngle)<3.0)
    //   pidController.setI(0.05);
    // else
    //   pidController.setI(0.001);
    controlleroutput = pidController.calculate(TargetAngle);

    // limit rotation speed of robot
    if (controlleroutput > 0.5)
    controlleroutput = 0.5;
    if (controlleroutput < -0.5)
    controlleroutput = -0.5;

    // turn robot towards target
    RobotContainer.drivetrain.drive(
      new Translation2d(0,0), controlleroutput * RobotContainer.drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
  
    // add time if we are on target within 1deg. Otherwise, reset timer
    if (RobotContainer.speakertargeting.IsTarget() && Math.abs(TargetAngle)<2.0)
      OnTargetTime += 0.02;
    else
      OnTargetTime = 0.0;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // switch off drive motors
    RobotContainer.drivetrain.drive(new Translation2d(0,0), 0.0, false);
    RobotContainer.operatorinterface.RobotAtAngle.setBoolean(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we are finished if locked on target for longer than 250ms
    return OnTargetTime >=0.1;
  }
}
