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
import edu.wpi.first.wpilibj.Timer;
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

  // Camera Notes Entry Array Definition
  // double[0] - number of notes - of 0 then no other data provided then array length=1
  // double[1] - x of center of nearest note
  // double[2] - y of center of nearest note
  // double[3] - x of center of 2nd nearest note
  // double[4] - y of center of 2nd nearest

  // is note detected in pickup camera?
  private boolean isNoteDetected;
  private double NoteX, NoteY;

  // holds the current number of april tags detected by the nvidia
  private int CurrentNumberDetections;

  private boolean m_isNvidiaAlive;
  private Timer m_NvidiaTimeoutTimer;
  private double m_NVidiaTimeout;

  /** Creates a new NVidia. */
  public NVidia() {
    m_table = NetworkTableInstance.getDefault().getTable("Nvidia");
    m_CameraSub3=m_table.getDoubleArrayTopic("camera1").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_CameraSub4=m_table.getDoubleArrayTopic("camera2").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_RobotPoseSub0=m_table.getDoubleArrayTopic("robot_pose_in_field2").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_RobotPoseSub1=m_table.getDoubleArrayTopic("robot_pose_in_field3").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_RobotPoseSub2=m_table.getDoubleArrayTopic("robot_pose_in_field4").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
    m_Notes=m_table.getDoubleArrayTopic("camera_notes").subscribe(null, PubSubOption.pollStorage(5), PubSubOption.periodic(0.02));
  
  
    isNoteDetected = false;

    // create timeout timer
    m_NvidiaTimeoutTimer = new Timer();
  }


  
  @Override
  public void periodic() {
    // array to hold apriltag detection data
    TimestampedDoubleArray AprilTagDetectionData[];

    int NumDetections = 0;

    // read data for camera 0
    AprilTagDetectionData= m_RobotPoseSub0.readQueue();
    NumDetections += AprilTagDetectionData.length;
    // for each AprilTag detection in the list
    for (int j=0;j<AprilTagDetectionData.length; j++) {
      data = AprilTagDetectionData[j].value;
      RobotContainer.odometry.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])), AprilTagDetectionData[j].value[3]);
    }

    // read data for camera 1
    AprilTagDetectionData= m_RobotPoseSub1.readQueue();
    NumDetections += AprilTagDetectionData.length;
    // for each AprilTag detection in the list
    for (int j=0;j<AprilTagDetectionData.length; j++) {
      data = AprilTagDetectionData[j].value;
      RobotContainer.odometry.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])), AprilTagDetectionData[j].value[3]);
    }

    // read data for camera 2
    AprilTagDetectionData= m_RobotPoseSub2.readQueue();
    NumDetections += AprilTagDetectionData.length;
    // for each AprilTag detection in the list
    for (int j=0;j<AprilTagDetectionData.length; j++){
      data = AprilTagDetectionData[j].value;
      RobotContainer.odometry.addVision(new Pose2d(data[0],data[1],new Rotation2d(data[2])), AprilTagDetectionData[j].value[3]);
    }

    // save number of apriltags currently detected
    CurrentNumberDetections = NumDetections;

    // look for notes
    TimestampedDoubleArray[] NotesData= m_Notes.readQueue();

    // do we have at least one note in picture?
    if (NotesData.length >0)
    {
      m_NvidiaTimeoutTimer.stop();
      m_NvidiaTimeoutTimer.reset();
      m_isNvidiaAlive=true;

      data = NotesData[0].value;

      if (data[0] > 0.1)
      {
        isNoteDetected = true;
        NoteX = data[1]-320.0; NoteY = data[2]-240.0;
      }
      else
      {
         // there is no note
        isNoteDetected = false;
      }
    }
    else
    {
      // start our NVidia timeout timer
        m_NvidiaTimeoutTimer.start();

        if (m_NvidiaTimeoutTimer.hasElapsed(m_NVidiaTimeout))
          m_isNvidiaAlive=false;
    }
    
  }

  // function returns the number of apriltags currently detected by the Nvidia
  public int GetNumberAprilTagsDetected()
  {
    return CurrentNumberDetections;
  }


  public boolean IsNoteDetected()
  { return isNoteDetected; }

  public double GetDetectedNoteX()
  { 
    if (isNoteDetected)
      return NoteX;
    else
     return 0.0;
  }

  public double GetDetectedNoteY()
  { 
    if (isNoteDetected)
     return NoteY;
    else
     return 0.0;
  }

  public boolean isNVidiaAlive()
  { return m_isNvidiaAlive;}


}