// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.RobotMap;
import frc.robot.util.ShuffleUser;
import frc.robot.util.SubsystemShuffleboardManager;


public class Pigeon extends SubsystemBase implements ShuffleUser {
  // subsystem shuffleboard controls
  private GenericEntry m_gyroPitch;
  private GenericEntry m_gyroYaw;
  private GenericEntry m_gyroRoll;

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
  public double getYawDeg() {
    
    // scaling factor for CTR Pigeon determine by test - Feb 5 2023
    double value = gyro.getYaw().getValue()*0.99895833 + OffsetAdjust;
    
    // convert continous number to -180 to +180deg to match NavX function call
    if (value > 0)
      return ((value+180.0) %360.0)-180.0;
    else
      return ((value-180.0) %360.0)+180.0;
  }

  public double getYawRad() {
    return getYawDeg()*Odometry.DEGtoRAD;
  }

  /** Gets the pitch of the robot
  * @return current pitch value in deg */
  public double getPitch() {
    
    return -gyro.getPitch().getValue();
  }

  /** Resets yaw to zero */
  public void resetGyro() {
    
    // reset our Gyro
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      OffsetAdjust = 180.0;
    } else {
      OffsetAdjust = 0.0;
    }
    gyro.reset();
  }

/** Resets yaw to a value */
  public void setGyro(double deg) {
    
    // reset our Gyro
    OffsetAdjust = 0.0;
    OffsetAdjust = -getYawDeg()+deg;
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
  public void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Pigeon");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Values", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 3);
    m_gyroPitch = l1.add("Pitch (deg)", 0.0).getEntry();
    m_gyroYaw = l1.add("Yaw (deg)", 0.0).getEntry();
    m_gyroRoll = l1.add("Roll (deg)", 0.0).getEntry();
    
  }

  /** Update subsystem shuffle board page with current Gyro values */
  public void updateShuffleboard() {
    // write current robot Gyro
    m_gyroPitch.setDouble(getPitch());
    m_gyroYaw.setDouble(getYawDeg());
    m_gyroRoll.setDouble(getRoll());
  }
}