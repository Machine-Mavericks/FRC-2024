// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.util.Utils;
import frc.robot.subsystems.Drivetrain;

public class ManualDriveCommand extends Command {

  private Drivetrain m_drivetrain;

  private PIDController m_headingPID = new PIDController(0.01, 0, 0);
  // Use Double class so it can be set to null
  private Double m_PIDTarget = null;
  private long m_pidDelay = -1;

  /** Creates a new DriveCommand. */
  public ManualDriveCommand(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PIDTarget = null;
    m_pidDelay = 10; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Driver inputs, should be in range [-1,1]
    //SmartDashboard.putNumber("SDB", SlowDownButton);

    double xInput = OI.getXDriveInput();
    double yInput = OI.getYDriveInput();
    double rotInput = -OI.getRotDriveInput();
    
    // ************* Functionality used for hanging only

    boolean useAlternateFieldReference = false;
    double AlternateFieldReferenceAngle = 0.0;
    
    if (OI.HangAButton.getAsBoolean())
    {
        useAlternateFieldReference = true;
        if (DriverStation.getAlliance().get() == Alliance.Red)
        {
          AlternateFieldReferenceAngle = -30.0;   // apears ok
          rotInput = -0.01*Utils.AngleDifference(-60.0, RobotContainer.odometry.getPose2d().getRotation().getDegrees());
        }
        else
          { AlternateFieldReferenceAngle = -30.0;   // DO NOT TOUCH
            rotInput = -0.01*Utils.AngleDifference(120.0, RobotContainer.odometry.getPose2d().getRotation().getDegrees());
          }
    }
    else if (OI.HangBButton.getAsBoolean())
    {
       useAlternateFieldReference = true;
       if (DriverStation.getAlliance().get() == Alliance.Red)
          {
            AlternateFieldReferenceAngle = 30.0;    // appears ok
            rotInput = -0.01*Utils.AngleDifference(60.0, RobotContainer.odometry.getPose2d().getRotation().getDegrees());
          }
        else
          {AlternateFieldReferenceAngle = 30.0;  // DO NOT TOUCH
          rotInput = -0.01*Utils.AngleDifference(-120.0, RobotContainer.odometry.getPose2d().getRotation().getDegrees());
          }
    }

    // ************* Functionality used for hanging only

    // if pressing target speaker button, then keep turning to speaker.
    else if (OI.constantTurnButton.getAsBoolean()) {
      rotInput = -RobotContainer.speakertargeting.getSpeakerAngle(RobotContainer.odometry.getPose2d())/60;
      if (rotInput>1) rotInput = 1;
    }

    // If no rotational input provided, use PID to hold heading
    // When the zero button is pressed force reset to prevent jumping
    if(rotInput == 0 && !OI.zeroButton.getAsBoolean()){
      if(m_pidDelay > 0) m_pidDelay --;
      else {
        // If the target is unset, set it to current heading
        if(m_PIDTarget == null){
          m_PIDTarget = RobotContainer.gyro.getYaw();
          m_headingPID.reset(); // Clear existing integral term as may accumulate while not in use
          //m_headingPID.setSetpoint(m_PIDTarget);
        }
        
        // if we are not moving robot, then apply 0 rotation speed to straighten wheels
        if (xInput==0.0 && yInput==0.0)
          rotInput = 0.0;
        else         // Compute rotational command from PID controller
          rotInput = m_headingPID.calculate(Utils.AngleDifference(m_PIDTarget, RobotContainer.gyro.getYaw()));
      }
    } else {
      // If there is input, set target to null so it's properly reset next time
      m_PIDTarget = null;
      m_pidDelay = 10; 
    }


    // if we are on red team, then rotate drive field drive by 180deg
    double dir = 1.0;
    if (DriverStation.getAlliance().get() == Alliance.Red) dir=-1.0;

    
    if (!useAlternateFieldReference)
        m_drivetrain.drive(new Translation2d(dir*yInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, dir*xInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND), rotInput*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true); 
    else
        m_drivetrain.drive(new Translation2d(dir*yInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, dir*xInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND), rotInput*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false, true, AlternateFieldReferenceAngle); 
      }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns false when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
