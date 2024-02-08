// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.AprilTagMap;

public class NVidia extends SubsystemBase {
  private int numCams = 4;
  private NetworkTable m_table;
  private DoubleArraySubscriber m_CameraSub[] = new DoubleArraySubscriber[4];

  /** Creates a new NVidia. */
  public NVidia() {
    // set pointer to limelight network table
    m_table = NetworkTableInstance.getDefault().getTable("Nvidia");
    // subscribe to Nvidia camera topics
    for (int i=0;i<numCams;i++){
      m_CameraSub[i]=m_table.getDoubleArrayTopic("camera"+Integer.toString(i)).subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    }
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
        RobotContainer.swervepose.addVision(AprilTagMap.CalculateRobotFieldPose(AprilTagDetectionData[j].value, i),  AprilTagDetectionData[j].timestamp);
     }
    }
  }
}