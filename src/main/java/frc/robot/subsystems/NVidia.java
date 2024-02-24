// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.AprilTagMap;

public class NVidia extends SubsystemBase {
  private int numCams = 2;
  private NetworkTable m_table;
  private DoubleArraySubscriber m_CameraSub[] = new DoubleArraySubscriber[4];
  private GenericEntry m_x;
  private GenericEntry m_y;
  private GenericEntry m_angle;
  private double[] data;

  /** Creates a new NVidia. */
  public NVidia() {
    // set pointer to limelight network table
    m_table = NetworkTableInstance.getDefault().getTable("Nvidia");
    // subscribe to Nvidia camera topics
    for (int i=0;i<numCams;i++){
      m_CameraSub[i]=m_table.getDoubleArrayTopic("camera"+Integer.toString(i)).subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    }
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // array to hold apriltag detection data
    TimestampedDoubleArray AprilTagDetectionData[];
    // get AprilTag detections from Netowrk Table for each camera
    for (int i=0;i<numCams;i++){
      AprilTagDetectionData= m_CameraSub[0].readQueue();
      // for each AprilTag detection in the list
      for (int j=0;j<AprilTagDetectionData.length; j++)
      {
        data = AprilTagDetectionData[j].value;
        RobotContainer.swervepose.addVision(AprilTagMap.CalculateRobotFieldPose(AprilTagDetectionData[j].value, i),  AprilTagDetectionData[j].timestamp, AprilTagDetectionData[j].value[8]);
     }
    }
    updateShuffleboard();
  }

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Swerve Estimator");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Estimates", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 3);
    m_x = l1.add("X (m)", 0.0).getEntry();
    m_y = l1.add("Y (m)", 0.0).getEntry();
    m_angle = l1.add("Angle(deg)", 0.0).getEntry();
  }   

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    m_x.setDouble(data[2]);
    m_y.setDouble(data[3]);
    m_angle.setDouble(data[5]);
  }

}