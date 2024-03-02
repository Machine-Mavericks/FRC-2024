// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class NVidia extends SubsystemBase {
  private NetworkTable m_table;
  private DoubleArraySubscriber m_CameraSub3;
  private DoubleArraySubscriber m_CameraSub4;
  private DoubleArraySubscriber m_RobotPoseSub3;
  private DoubleArraySubscriber m_RobotPoseSub4;
  private GenericEntry m_x;
  private GenericEntry m_y;
  private GenericEntry m_angle;
  private double[] data;

  /** Creates a new NVidia. */
  public NVidia() {
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    if (NetworkTableInstance.getDefault().hasSchema("Nvidia")){
      // set pointer to limelight network table
      m_table = NetworkTableInstance.getDefault().getTable("Nvidia");
      // array to hold apriltag detection data
      TimestampedDoubleArray AprilTagDetectionData[];
      // subscribe to Nvidia camera topics
      if (m_table.containsSubTable("camera3")) {
        m_CameraSub3=m_table.getDoubleArrayTopic("camera3").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
      }
      if (m_table.containsSubTable("camera4")){
        m_CameraSub4=m_table.getDoubleArrayTopic("camera4").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
      }
      if (m_table.containsSubTable("robot_pose_in_field3")){
        m_RobotPoseSub3=m_table.getDoubleArrayTopic("robot_pose_in_field3").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
        // get AprilTag detections from Netowrk Table for each camera
        AprilTagDetectionData= m_RobotPoseSub3.readQueue();
        // for each AprilTag detection in the list
        for (int j=0;j<AprilTagDetectionData.length; j++) {
          data = AprilTagDetectionData[j].value;
          //RobotContainer.swervepose.addVision(AprilTagMap.CalculateRobotFieldPose(data, 1),  AprilTagDetectionData[j].timestamp, AprilTagDetectionData[j].value[8]);
          RobotContainer.swervepose.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])),  AprilTagDetectionData[j].timestamp, AprilTagDetectionData[j].value[3]);
        }
      }
      if (m_table.containsSubTable("robot_pose_in_field3")){
        m_RobotPoseSub4=m_table.getDoubleArrayTopic("robot_pose_in_field4").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
        AprilTagDetectionData= m_RobotPoseSub4.readQueue();
        // for each AprilTag detection in the list
        for (int j=0;j<AprilTagDetectionData.length; j++)
        {
          data = AprilTagDetectionData[j].value;
          //RobotContainer.swervepose.addVision(AprilTagMap.CalculateRobotFieldPose(data, 1),  AprilTagDetectionData[j].timestamp, AprilTagDetectionData[j].value[8]);
          RobotContainer.swervepose.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])),  AprilTagDetectionData[j].timestamp, AprilTagDetectionData[j].value[3]);
        }
      }
      updateShuffleboard(data);
    }
    
  }

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("NVidia");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("AprilTags", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 3);
    m_x = l1.add("X (m)", 0.0).getEntry();
    m_y = l1.add("Y (m)", 0.0).getEntry();
    m_angle = l1.add("Angle(deg)", 0.0).getEntry();
  }   

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard(double[] data) {
    m_x.setDouble(data[2]);
    m_y.setDouble(data[3]);
    m_angle.setDouble(data[5]);
  }

}