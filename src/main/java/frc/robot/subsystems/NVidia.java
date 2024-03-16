// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class NVidia extends SubsystemBase {
  private NetworkTable m_table;
  private DoubleArraySubscriber m_CameraSub3;
  private DoubleArraySubscriber m_CameraSub4;
  private DoubleArraySubscriber m_RobotPoseSub0;
  private DoubleArraySubscriber m_RobotPoseSub1;
  private DoubleArraySubscriber m_RobotPoseSub2;
  private DoubleArraySubscriber m_Notes;
  private double[] data;

  /** Creates a new NVidia. */
  public NVidia() {
    m_table = NetworkTableInstance.getDefault().getTable("Nvidia");
    m_CameraSub3=m_table.getDoubleArrayTopic("camera1").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_CameraSub4=m_table.getDoubleArrayTopic("camera2").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_RobotPoseSub0=m_table.getDoubleArrayTopic("robot_pose_in_field2").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_RobotPoseSub1=m_table.getDoubleArrayTopic("robot_pose_in_field3").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_RobotPoseSub2=m_table.getDoubleArrayTopic("robot_pose_in_field4").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_Notes=m_table.getDoubleArrayTopic("camera_notes").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
  }

  @Override
  public void periodic() {
    // array to hold apriltag detection data
    TimestampedDoubleArray AprilTagDetectionData[];

    // read data for camera 0
    AprilTagDetectionData= m_RobotPoseSub0.readQueue();
    // for each AprilTag detection in the list
    for (int j=0;j<AprilTagDetectionData.length; j++) {
      data = AprilTagDetectionData[j].value;
      RobotContainer.odometry.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])), AprilTagDetectionData[j].value[3]);
    }

    // read data for camera 1
    AprilTagDetectionData= m_RobotPoseSub1.readQueue();
    // for each AprilTag detection in the list
    for (int j=0;j<AprilTagDetectionData.length; j++) {
      data = AprilTagDetectionData[j].value;
      RobotContainer.odometry.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])), AprilTagDetectionData[j].value[3]);
    }

    // read data for camera 2
    AprilTagDetectionData= m_RobotPoseSub2.readQueue();
    // for each AprilTag detection in the list
    for (int j=0;j<AprilTagDetectionData.length; j++){
      data = AprilTagDetectionData[j].value;
      RobotContainer.odometry.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])), AprilTagDetectionData[j].value[3]);
    }

    TimestampedDoubleArray[] NotesData= m_Notes.readQueue();
  }
}