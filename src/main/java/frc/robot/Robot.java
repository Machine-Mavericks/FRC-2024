// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drive.ManualDriveCommand;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.util.SubsystemShuffleboardManager;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  // boolean flag to indicate if robot has been initialized
  boolean robotIsInitialized = false;
  
  private Command autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    RobotMap.Init();

    SubsystemShuffleboardManager.init(this);
    
    // Initialise our RobotContainer. This will perform all our button bindings, and
    // put our
    // autonomous chooser on the dashboard.
    RobotContainer.init();

    // reset the gyro when robot powered up
    RobotContainer.gyro.resetGyro();

    // measure angle based on odometry
    RobotContainer.odometry.setCamBool(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
   
    // robot is now initialized
    robotIsInitialized = true;
    autonomousCommand = RobotContainer.getAutonomousCommand();
    RobotContainer.cassetteangle.setAngle(CassetteEffector.DROP_PROP_ANGLE);

    RobotContainer.gyro.setGyroUsingOdom();
    
    // switch odometry modes before auto sequence
    RobotContainer.odometry.setCamBool(false);
    
    // schedule the autonomous command
    if (autonomousCommand != null)
      autonomousCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
   

    // if robot has not previously been initialized, then go ahead and initialize
    //if (robotIsInitialized == false) {
      // RobotContainer.gyro.resetGyro();
      // RobotContainer.odometry.InitializetoZero();
      
      // robot is now initialized
      //robotIsInitialized = true;
    //}

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    RobotContainer.gyro.setGyroUsingOdom();
    // set default swerve drive command to manual drive mode
    RobotContainer.drivetrain.setDefaultCommand(new ManualDriveCommand(RobotContainer.drivetrain));

    // stop angle based on odometry
    RobotContainer.odometry.setCamBool(false);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    RobotContainer.gyro.resetGyro();
  }

  /** This function is called 
  periodically during test mode. */
  @Override
  public void testPeriodic() {

    
  }
}
