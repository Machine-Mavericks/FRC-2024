// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SpeakerTargeting;
import frc.robot.util.Utils;

public class AimToSpeaker extends Command {
  private final SpeakerTargeting speakertargeting;

  // PID gains for rotating robot towards target
  double kp = 0.019;
  double ki = 0.0;
  double kd = 0.001;
  PIDController pidController = new PIDController(kp, ki, kd);
  
  // timer counts how long robot is lined up to target for
  double OnTargetTime;
  
  /** Creates a new AimToSpeaker */
  public AimToSpeaker() {
    // aiming uses drive system, angle system, and spins up the shooter
    addRequirements(RobotContainer.drivetrain, RobotContainer.cassetteangle, RobotContainer.cassetteshooter);

    speakertargeting = RobotContainer.speakertargeting;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset the PID controller
    pidController.reset();

    TargetAngle=0.0;
  }

  // angle to target
  double TargetAngle;
  int missedSamples;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if we have target, then get angle. If no target, assume 0deg
    if ((speakertargeting.IsTarget()))
    {
      TargetAngle = Utils.AngleDifference(speakertargeting.getSpeakerAngle(), RobotContainer.gyro.getYawDeg());
      System.out.println("Angle is " + TargetAngle);
      missedSamples = 0;
    }
    else{
      missedSamples +=1;
      if (missedSamples >=3)
        TargetAngle = 0;
    }
      

    // calculate PID controller
    double angleControlleroutput = 0.0;

    angleControlleroutput = pidController.calculate(TargetAngle);

    // limit rotation speed of robot
    if (angleControlleroutput > 0.2)
    angleControlleroutput = 0.2;
    if (angleControlleroutput < -0.2)
    angleControlleroutput = -0.2;

    // Joystick input
    double xInput = OI.getXDriveInput();
    double yInput = OI.getYDriveInput();

    // turn robot towards target
    RobotContainer.drivetrain.drive(
      new Translation2d(yInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, xInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND), angleControlleroutput * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);
  
    // spinup shooter
    RobotContainer.cassetteshooter.leftShootRun(speakertargeting.getDesiredLSpeed());
    RobotContainer.cassetteshooter.rightShootRun(speakertargeting.getDesiredRSpeed());

    // set cassette angle
    RobotContainer.cassetteangle.setAngle(speakertargeting.getDesiredAngle());
    
    if (xInput == 0 && yInput == 0) { // Fine to check if zero because of deadzones
      if (speakertargeting.IsAligned() && speakertargeting.IsSpunUp())
        OnTargetTime += RobotContainer.updateDt;
      else
        OnTargetTime = 0.0;
      }
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // switch off drive motors
    RobotContainer.drivetrain.drive(new Translation2d(0,0), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we are finished if locked on target for longer than 250ms
    return OnTargetTime >= 0.1;
  }
}
