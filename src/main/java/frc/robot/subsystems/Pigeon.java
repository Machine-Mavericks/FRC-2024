// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.util.ShuffleUser;
import frc.robot.util.SubsystemShuffleboardManager;


public class Pigeon extends SubsystemBase implements ShuffleUser {

  //gyro offset adjust
  private double OffsetAdjust;

  // make our gyro object
  private static Pigeon2 gyro;

  /** Creates a new Gyro. */
  public Pigeon() {
    // initialize shuffleboard
    SubsystemShuffleboardManager.RegisterShuffleUser(this, true, 5);
    
    // make pigeon object
    gyro = new Pigeon2(RobotMap.CANID.PIGEON, Drivetrain.CAN_BUS_NAME);

    // offset adjust
    OffsetAdjust = 0.0;
  }

  @Override
  public void periodic() {
  }

  /** Gets the yaw of the robot
   * @return current yaw value (-180 to 180) */
  public double getYaw() {
    
    // scaling factor for CTR Pigeon determine by test - Feb 5 2023
    double value = gyro.getYaw().getValue()*0.99895833 + OffsetAdjust;
    
    // convert continous number to -180 to +180deg to match NavX function call
    if (value > 0)
      return ((value+180.0) %360.0)-180.0;
    else
      return ((value-180.0) %360.0)+180.0;
  }

  /** Gets the pitch of the robot
  * @return current pitch value in deg */
  public double getPitch() {
    
    return -gyro.getPitch().getValue();
  }

  /** Resets yaw to zero -
   * reset angle depends on team alliance as gyro will be pointed in field direction */
  public void resetGyro() {
    
    // reset our Gyro - if on red team, we reset to 180deg
    if (DriverStation.getAlliance().get() == Alliance.Red)
    {
      OffsetAdjust = 180.0;
    } 
    else
    {
      OffsetAdjust = 0.0;
    }
    gyro.reset();
  }

  /**
   * sets gyro using odometry and alliance
   */
  public void setGyroUsingOdom() {
    
    // get our location
    Pose2d mylocation = RobotContainer.odometry.getPose2d();
    double newGyroAngle = mylocation.getRotation().getDegrees();

    //if (DriverStation.getAlliance().get() == Alliance.Blue) {
      this.setGyro(newGyroAngle);
      RobotContainer.odometry.setPosition(mylocation.getX(), mylocation.getY(), newGyroAngle, newGyroAngle);
      
    //} else {
    //  this.setGyro(newGyroAngle+180);
    //  RobotContainer.odometry.setPosition(mylocation.getX(), mylocation.getY(), newGyroAngle+180.0, newGyroAngle+180.0);
    //}
  }

/** Resets yaw to zero -
   * reset angle depends on team alliance as gyro will be pointed in field direction */
  public void resetGyroReverse() {
    
    // reset our Gyro - if on red team, we reset to 180deg
    if (DriverStation.getAlliance().get() == Alliance.Blue)
    {
      OffsetAdjust = 180.0;
    } 
    else
    {
      OffsetAdjust = 0.0;
    }
    gyro.reset();
  }


/** Resets yaw to a value */
  public void setGyro(double deg) {
    
    // reset our Gyro
    OffsetAdjust = 0.0;
    OffsetAdjust = -getYaw()+deg;
  }


    /**
   * Gets signal latency for the gyro's yaw
   */
  public double getYawLatency(){
    return gyro.getYaw().getTimestamp().getLatency();
  }


  /** Accumulated yaw
   * @return accumulated angle in degrees */
  public double continuousYaw() {
    return gyro.getAngle();
  }

  /** Get Roll
   * @return roll in deg */
  public double getRoll() {
    return gyro.getRoll().getValue();
  }


  /** Gyro Shuffleboard */

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  public void initializeShuffleboard() {}

  /** Update subsystem shuffle board page with current Gyro values */
  public void updateShuffleboard() {
    // write current robot Gyro
    RobotContainer.operatorinterface.m_gyroPitch.setDouble(getPitch());
    RobotContainer.operatorinterface.m_gyroYaw.setDouble(getYaw());
    RobotContainer.operatorinterface.m_gyroRoll.setDouble(getRoll());
  }

}