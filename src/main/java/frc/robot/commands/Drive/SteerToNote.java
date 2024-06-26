// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.subsystems.Drivetrain;

public class SteerToNote extends Command {
  // subsystems we are interfacing with
  private Drivetrain m_drivetrain = RobotContainer.drivetrain;

  private static final double MOVEMENT_ALIGNMENT_MULTIPLIER = 2.4;

  // angle to target
  double TargetAngle = 0;

  // PID gains for rotating robot towards note target
  double kp = 0.005;
  double ki = 0.0;
  double kd = 0.0;
  PIDController pidController = new PIDController(kp, ki, kd);

  // is this command automated or semi-automated?
  boolean m_automated;

  // speed limit when automated (0<speed<1.0)
  double m_speedLimitAuto;

  // command timeout time
  double m_timeoutlimit;
  Timer timer;

  /** Steers robot towards note
   * Input: true if fully automated, false if only sem-automated */
  public SteerToNote(boolean automated, double timeout) {
    // this command requires use of drivetrain and gyro
    addRequirements(m_drivetrain, RobotContainer.cassetteangle);
    
    m_automated = automated;
    m_timeoutlimit = timeout;

    m_speedLimitAuto = 0.3;
    timer = new Timer();
  }
  
  /** Steers robot towards note
   * Input: true if fully automated, false if only sem-automated,
   *        speed limit (0 to 1.0) if automated */
  public SteerToNote(boolean automated, double timeout, double speedlimit) {
    // this command requires use of drivetrain
    addRequirements(m_drivetrain);

    m_automated = automated;
    m_timeoutlimit = timeout;
    m_speedLimitAuto=  speedlimit;

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cassetteangle.setAngle(CassetteEffector.GROUND_ANGLE);
    RobotContainer.cassetteintake.intakeRun(1);

    TargetAngle = 0;

    // reset pid controller
    pidController.reset();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if automated, assume 50% speed, in manual get speed from joystick
    double xInput = 0;

    if (m_automated) {
      xInput = m_speedLimitAuto;
    }else{
      xInput = OI.getYDriveInput();
      if (xInput < 0) {
        xInput = 0;
      }
    }

    // assume sideway speed of 0% unless determined otherwise
    double yInput = 0.0;
    
    // assume rotation not needed unless proven otherwise
    double rotate = 0.0;

    // do we have a valid target? 
    if ((RobotContainer.notetargeting.IsTarget())){
      TargetAngle = RobotContainer.notetargeting.getNoteHorAngle();
    
      // determine angle correction - uses PI controller
      // limit rotation to +/- 100% of available speed
      rotate = pidController.calculate(TargetAngle);
      if (rotate > 1.0)
        rotate = 1.0;
      if (rotate < -1.0)
        rotate = -1.0;

      // if not fully automatic, get joystick inputs
      if (m_automated)
      {
        // slow down forward speed if large angle to allow robot to turn without moving off course

        xInput = xInput * Math.abs(Math.cos(Math.toRadians(TargetAngle) * MOVEMENT_ALIGNMENT_MULTIPLIER));
      }
    }
    // command robot to drive - using robot-relative coordinates
    RobotContainer.drivetrain.drive(
      new Translation2d(xInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          yInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
          rotate * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.cassetteintake.intakeRun(0);
    RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE);

    RobotContainer.drivetrain.drive(new Translation2d(0,0), 0, false);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(m_timeoutlimit) || RobotContainer.cassetteintake.NoteOrNoNote();
  }
}
