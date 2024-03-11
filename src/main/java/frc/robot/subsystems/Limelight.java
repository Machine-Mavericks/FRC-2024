// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {
  /** Creates a new limelight. */

    // network table to communicate to camera with
    private NetworkTable m_table;

    // subsystem shuffleboard controls
    private GenericEntry m_Pipeline;
    private GenericEntry m_TargetPresent;
    private GenericEntry m_AprilTagID;

    //private DoubleArraySubscriber m_llpythonSub;

    private GenericEntry m_AngleX;
    private GenericEntry m_AngleY;
    private GenericEntry m_Skew;
    private GenericEntry m_Area;
    private GenericEntry m_Short;
    private GenericEntry m_Long;
    private GenericEntry m_Hor;
    private GenericEntry m_Vert;
    private GenericEntry m_X;
    private GenericEntry m_Y;
    private GenericEntry m_Z;
    private GenericEntry m_Pitch;
    private GenericEntry m_Yaw;
    private GenericEntry m_Roll;

    private GenericEntry m_Xfs;
    private GenericEntry m_Yfs;
    private GenericEntry m_Zfs;
    private GenericEntry m_Pitchfs;
    private GenericEntry m_Yawfs;
    private GenericEntry m_Rollfs;
    private GenericEntry m_Xrs;
    private GenericEntry m_Yrs;
    private GenericEntry m_Zrs;
    private GenericEntry m_Pitchrs;
    private GenericEntry m_Yawrs;
    private GenericEntry m_Rollrs;

    
    // true if fiducial information is enabled and to be displayed on shuffleboard
    private boolean m_FiducialEnable;
    private GenericEntry m_BotPose[] = new GenericEntry[6];
    private GenericEntry m_BotPoseRed[] = new GenericEntry[6];
    private GenericEntry m_BotPoseBlue[] = new GenericEntry[6];

    private LimelightResults cached_json_results = new LimelightResults();
    private boolean cached_data_valid = false;
  
    /**
     * Creates a new Limelight.
     * Input: String containing name of limelight (defined in the camera)
     */
    public Limelight(String name, boolean FiducialEnable) {
      ConstructLimelight(name, FiducialEnable); }
    public Limelight(String name) {
      ConstructLimelight(name, false); }

    
    // construct limelight subsystem
    private void ConstructLimelight(String name, boolean FiducialEnable) {
      
      // record fiducial enable
      m_FiducialEnable = FiducialEnable;
    
      // set pointer to limelight network table
      m_table = NetworkTableInstance.getDefault().getTable("limelight-"+name);
      
      // initialize camera to use LED mode set in the current pipeline setup
      m_table.getEntry("ledMode").setNumber(0);

      // set camera streaming mode - primary and secondary cameras are placed
      // side-by-side
      m_table.getEntry("stream").setNumber(0);

      //m_llpythonSub = m_table.getDoubleArrayTopic("llpython").subscribe(new double[] {});
    
      // set initial pipeline to 0
      setPipeline(0);
  
      // create shuffleboard page
      initializeShuffleboard(name);
    }


    // This method will be called once per scheduler run
    int m_UpdateTimer = 0;
    @Override
    public void periodic() {
      // update shuffleboard - update at 5Hz is sufficient for this subsystem
      m_UpdateTimer++;
      if (m_UpdateTimer>=15)
      {
        updateShuffleboard();
        m_UpdateTimer=0;
      }

      // Invalidate cached data for next command loop
      cached_data_valid = false;
    }
  
    // ---------- Camera Control Functions ----------
  
    /** set camera's current pipeline: 0 to 9 */

    public void setPipeline(int num) {

      if (num >= 0 && num <= 9)
      m_table.getEntry("pipeline").setNumber(num);
    }
  
    /** returns camera's current pipeline: 0 to 9 */

    public Double getPipeline() {

      return m_table.getEntry("getPipe").getDouble(0);
    }
  
    // ---------------Camera Access Functions ---------------------
  
    /**
     * get horitaonal angle from center of camera view to center of target
     * returns -27 to +27 degrees
     */
    public float getHorizontalTargetOffsetAngle() {
      return m_table.getEntry("tx").getFloat(0);
    }
  
    /**
     * get vertical angle from center of camera view to center of target
     * returns -20 to +20 degrees
     */
     public float getVerticalTargetOffsetAngle() {
      
      return m_table.getEntry("ty").getFloat(0);
    }
  
    /** get rotation angle between view of camera of target */
    float getTargetSkew() {
      return m_table.getEntry("ts").getFloat(0);
    }
  
    // get target detection time latency
    public double getLatencyContribution() {
      return m_table.getEntry("tl").getDouble(0);
    }
  
    /** get whether target is currently detected or not, returns 0 or 1 */ 
    public boolean isTargetPresent() {
      double temp = m_table.getEntry("tv").getDouble(0);
      return (temp>0.05);
    }
  
    /** get target area atributes - 0 to 100% of image */
    public double getTargetArea() {
      return m_table.getEntry("ta").getDouble(0);
    }
  
    /** returns shortest sidelength of target fitted bounding box (# pixels) */
    public double getShortestSide() {
      return m_table.getEntry("tshort").getDouble(0);
    }
  
    /** returns longest sidelength of target fitted bounding box (# pixels) */
    public double getLongestSide() {
      return m_table.getEntry("tlong").getDouble(0);
    }
  
    /** Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
    public double getHorizontalSideLength() {
      return m_table.getEntry("thor").getDouble(0);
    }
  
    /** Vertical sidelength of the rough bounding box (0 - 320 pixels) */
    public double getVerticalSideLength() {
      return m_table.getEntry("tvert").getDouble(0);
    }
  
    /** Get primary april tag id */
    public double getPrimAprilTagID () {
      return m_table.getEntry("tid").getDouble(0);
    }

    // ---------- get raw target attributes ----------
  
    /** Raw Screenspace X */
    float getRawScreenspaceX0()
    {  return m_table.getEntry("tx0").getFloat(0); }
  
    /** Raw Screenspace X */
    float getRawScreenspaceX1()
    {  return m_table.getEntry("tx1").getFloat(0); }
  
    /** Raw Screenspace X */
    float getRawScreenspaceX2()
    {  return m_table.getEntry("tx2").getFloat(0); }
  
    /** Raw Screenspace Y */
    float getRawScreenspaceY0()
    {  return m_table.getEntry("ty0").getFloat(0);}
  
    /** Raw Screenspace Y */
    float getRawScreenspaceY1()
    {  return m_table.getEntry("ty1").getFloat(0); }
  
    /** Raw Screenspace Y */
    float getRawScreenspaceY2()
    {  return m_table.getEntry("ty2").getFloat(0); }
  
    /** Area (0% of image to 100% of image) */
    float getRawArea0()
    {  return m_table.getEntry("ta0").getFloat(0);}
  
    /** Area (0% of image to 100% of image) */
    float getRawArea1()
    {  return m_table.getEntry("ta1").getFloat(0);}
  
    /** Area (0% of image to 100% of image) */
    float getRawArea2()
    {  return m_table.getEntry("ta2").getFloat(0);}
  
    /** Skew or rotation (-90 degrees to 0 degrees) */
    float getRawSkew0()
    {  return m_table.getEntry("ts0").getFloat(0);}
  
    /** Skew or rotation (-90 degrees to 0 degrees) */
    float getRawSkew1()
    {  return m_table.getEntry("ts1").getFloat(0);}
  
    /** Skew or rotation (-90 degrees to 0 degrees) */
    float getRawSkew2()
    { return m_table.getEntry("ts2").getFloat(0);}
  

    // -------------------- Apriltag Functions --------------------
    
    /* get Robot pose (field space) */
    public Pose3d getBotPose() {
      return GetPose3dFromCamera("botpose");
    }
      
    /* get Robot pose (field space) */
    public Pose3d getBotPoseBlue() {
      return GetPose3dFromCamera("botpose_wpiblue");
    }
    
    /* get Robot pose (field space) */
    public Pose3d getBotPoseRed() {
      return GetPose3dFromCamera("botpose_wpired");
    }

    public void addDetection() {
      // get vector from camera. If valid length, convert to Pose3d
      double[] vector = m_table.getEntry("botpose").getDoubleArray(new double[]{});  
    
      // if vector is valid (has 6 numbers in it) go ahead and record data in structure
      if (vector.length>=6){
        RobotContainer.swervepose.addVision(new Pose2d(vector[0]+8.24,vector[1]+4.05,new Rotation2d(vector[3])), getTargetArea());
      }
    }
    
    /** Generic helper function to get Pose3d vector from camera given
         input name: name of vector to retrieve */
    private Pose3d GetPose3dFromCamera(String name) {
      
      // get vector from camera. If valid length, convert to Pose3d
      double[] vector = m_table.getEntry(name).getDoubleArray(new double[]{});  
      
      // if vector is valid (has 6 numbers in it) go ahead and record data in structure
      if (vector.length<6)
        return new Pose3d();
      else 
        return new Pose3d(
              new Translation3d(vector[0], vector[1], vector[2]),
              new Rotation3d(Units.degreesToRadians(vector[3]), Units.degreesToRadians(vector[4]),
                Units.degreesToRadians(vector[5])));
        }


  // -------------------- Subsystem Shuffleboard Methods --------------------


  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard(String name) {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Limelight: "+name);

    // camera pipeline number
    m_Pipeline = Tab.add("Pipeline", 0)
                    .withPosition(0,0).getEntry();

    // does camera detect target
    m_TargetPresent = Tab.add("Target Present", false).withPosition(0,1).getEntry();

    // april tag target id
    m_AprilTagID = Tab.add("AprilTag Target ID", 0).withPosition(0,2).getEntry();

    // camera target information
    ShuffleboardLayout l1 = Tab.getLayout("Target", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(1, 4);
    m_AngleX = l1.add("AngleX", 0.0).getEntry();
    m_AngleY = l1.add("AngleY", 0.0).getEntry();
    m_Skew = l1.add("Skew", 0.0).getEntry(); 
    
    // target dimensions
    ShuffleboardLayout l2 = Tab.getLayout("Dimensions", BuiltInLayouts.kList);
    l2.withPosition(3,0);
    l2.withSize(1,4);
    m_Area = l2.add("Area", 0.0).getEntry(); 
    m_Short = l2.add("Short", 0.0).getEntry(); 
    m_Long = l2.add("Long", 0.0).getEntry(); 
    m_Hor = l2.add("Hor", 0.0).getEntry(); 
    m_Vert = l2.add("Vert", 0.0).getEntry();

    // only show Fiducial information if requested
    if (m_FiducialEnable) {
      ShuffleboardLayout l3 = Tab.getLayout("BotPose", BuiltInLayouts.kList);
      l3.withPosition(4,0);
      l3.withSize(1,4);
      m_BotPose[0] = l3.add("x", 0.0).getEntry();
      m_BotPose[1] = l3.add("y", 0.0).getEntry(); 
      m_BotPose[2] = l3.add("z", 0.0).getEntry(); 
      m_BotPose[3] = l3.add("rx", 0.0).getEntry(); 
      m_BotPose[4] = l3.add("ry", 0.0).getEntry(); 
      m_BotPose[5] = l3.add("rz", 0.0).getEntry();
      
      ShuffleboardLayout l4 = Tab.getLayout("BotPose Blue", BuiltInLayouts.kList);
      l4.withPosition(5,0);
      l4.withSize(1,4);
      m_BotPoseBlue[0] = l4.add("x", 0.0).getEntry();
      m_BotPoseBlue[1] = l4.add("y", 0.0).getEntry(); 
      m_BotPoseBlue[2] = l4.add("z", 0.0).getEntry(); 
      m_BotPoseBlue[3] = l4.add("rx", 0.0).getEntry(); 
      m_BotPoseBlue[4] = l4.add("ry", 0.0).getEntry(); 
      m_BotPoseBlue[5] = l4.add("rz", 0.0).getEntry();

      ShuffleboardLayout l5 = Tab.getLayout("BotPose Red", BuiltInLayouts.kList);
      l5.withPosition(6,0);
      l5.withSize(1,4);
      m_BotPoseRed[0] = l5.add("x", 0.0).getEntry();
      m_BotPoseRed[1] = l5.add("y", 0.0).getEntry(); 
      m_BotPoseRed[2] = l5.add("z", 0.0).getEntry(); 
      m_BotPoseRed[3] = l5.add("rx", 0.0).getEntry(); 
      m_BotPoseRed[4] = l5.add("ry", 0.0).getEntry(); 
      m_BotPoseRed[5] = l5.add("rz", 0.0).getEntry();
    
      // temporary for testing 
      // april tag target id
      // m_Test1 = Tab.add("Num Targets", 0).withPosition(0,3).getEntry();
      // m_Test2 = Tab.add("Target ID", 0).withPosition(0,4).getEntry();
      // end temp ////////
    }

  }


  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    
    // update camera pipeline and target detected indicator
    m_Pipeline.setDouble(getPipeline());
    m_TargetPresent.setBoolean(isTargetPresent());
    m_AprilTagID.setDouble(getPrimAprilTagID());

    // update angles to center of target
    m_AngleX.setDouble(getHorizontalTargetOffsetAngle());
    m_AngleY.setDouble(getVerticalTargetOffsetAngle());
    m_Skew.setDouble(getTargetSkew());

    // update dimensions of target
    m_Area.setDouble(getTargetArea());
    m_Short.setDouble(getShortestSide());
    m_Long.setDouble(getLongestSide());
    m_Hor.setDouble(getHorizontalSideLength());
    m_Vert.setDouble(getVerticalSideLength());

    // only show Fiducial information if requested
    if (m_FiducialEnable) {
      // update robot position in field space
      Pose3d pose = getBotPose();
      m_BotPose[0].setDouble(pose.getTranslation().getX());
      m_BotPose[1].setDouble(pose.getTranslation().getY());
      m_BotPose[2].setDouble(pose.getTranslation().getZ());
      m_BotPose[3].setDouble(Units.radiansToDegrees(pose.getRotation().getX()));
      m_BotPose[4].setDouble(Units.radiansToDegrees(pose.getRotation().getY()));
      m_BotPose[5].setDouble(Units.radiansToDegrees(pose.getRotation().getZ()));

      // update target position in blue space
      pose = getBotPoseBlue();
      m_BotPoseBlue[0].setDouble(pose.getTranslation().getX());
      m_BotPoseBlue[1].setDouble(pose.getTranslation().getY());
      m_BotPoseBlue[2].setDouble(pose.getTranslation().getZ());
      m_BotPoseBlue[3].setDouble(Units.radiansToDegrees(pose.getRotation().getX()));
      m_BotPoseBlue[4].setDouble(Units.radiansToDegrees(pose.getRotation().getY()));
      m_BotPoseBlue[5].setDouble(Units.radiansToDegrees(pose.getRotation().getZ()));

      // update target position in blue space
      pose = getBotPoseRed();
      m_BotPoseRed[0].setDouble(pose.getTranslation().getX());
      m_BotPoseRed[1].setDouble(pose.getTranslation().getY());
      m_BotPoseRed[2].setDouble(pose.getTranslation().getZ());
      m_BotPoseRed[3].setDouble(Units.radiansToDegrees(pose.getRotation().getX()));
      m_BotPoseRed[4].setDouble(Units.radiansToDegrees(pose.getRotation().getY()));
      m_BotPoseRed[5].setDouble(Units.radiansToDegrees(pose.getRotation().getZ()));

      // temporary for testing purposes
      //LimelightResults results = GetJSONResults();
      //m_Test1.setDouble(results.targetingResults.targets_Fiducials.length);
      //m_Test2.setDouble(results.targetingResults.targets_Fiducials[0].fiducialID);
    }
  }


// -------------------- Fiducial Classes for JSON File Access and Parsing --------------------


// gets JSON fiducial target into from camera
private static ObjectMapper mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);;

/**
 * Gets latest JSON dump from limelight, calling more than once per loop is fine since results are cached
 */
public LimelightResults getLatestJSONDump(){
  if (!cached_data_valid) {
    UpdateJSONResults();
    cached_data_valid = true;
  }
  return cached_json_results;
}

private void UpdateJSONResults()
{
  cached_json_results = new LimelightResults();
  // get json from camera
  String rawData = m_table.getEntry("json").getString("");
  
  try {
    cached_json_results = mapper.readValue(rawData, LimelightResults.class);
  } catch (JsonProcessingException e) {
    System.err.println("lljson error: " + e.getMessage());
  }
}

// structure containing JSON results
public static class LimelightResults {
  @JsonProperty("Results")
  public Results targetingResults;

  public LimelightResults() {
      targetingResults = new Results();
  }
}

public static class Results {
  @JsonProperty("Fiducial")
  public LimelightTarget_Fiducial[] targets_Fiducials;

  public Results() {
      targets_Fiducials = new LimelightTarget_Fiducial[0];
  }
}

// camera information about each fiducial target
public static class LimelightTarget_Fiducial {

  @JsonProperty("fID")
  public double fiducialID;

  @JsonProperty("fam")
  public String fiducialFamily;

  @JsonProperty("t6c_ts")
  public double[] cameraPose_TargetSpace;

  @JsonProperty("t6r_fs")
  public double[] robotPose_FieldSpace;

  @JsonProperty("t6r_ts")
  public double[] robotPose_TargetSpace;

  @JsonProperty("t6t_cs")
  public double[] targetPose_CameraSpace;

  @JsonProperty("t6t_rs")
  public double[] targetPose_RobotSpace;

  /*public Pose3d getCameraPose_TargetSpace()
  { return toPose3D(cameraPose_TargetSpace); }
  public Pose3d getRobotPose_FieldSpace()
  { return toPose3D(robotPose_FieldSpace); }
  public Pose3d getRobotPose_TargetSpace()
  { return toPose3D(robotPose_TargetSpace); }
  public Pose3d getTargetPose_CameraSpace()
  { return toPose3D(targetPose_CameraSpace); }
  public Pose3d getTargetPose_RobotSpace()
  { return toPose3D(targetPose_RobotSpace);}
  public Pose2d getCameraPose_TargetSpace2D()
  { return toPose2D(cameraPose_TargetSpace); }
  public Pose2d getRobotPose_FieldSpace2D()
  { return toPose2D(robotPose_FieldSpace); }
  public Pose2d getRobotPose_TargetSpace2D()
  { return toPose2D(robotPose_TargetSpace); }
  public Pose2d getTargetPose_CameraSpace2D()
  { return toPose2D(targetPose_CameraSpace); }
  public Pose2d getTargetPose_RobotSpace2D()
  {  return toPose2D(targetPose_RobotSpace);} */
  
  @JsonProperty("ta")
  public double ta;

  @JsonProperty("tx")
  public double tx;

  @JsonProperty("txp")
  public double tx_pixels;

  @JsonProperty("ty")
  public double ty;

  @JsonProperty("typ")
  public double ty_pixels;

  @JsonProperty("ts")
  public double ts;
  
  public LimelightTarget_Fiducial() {
      cameraPose_TargetSpace = new double[6];
      robotPose_FieldSpace = new double[6];
      robotPose_TargetSpace = new double[6];
      targetPose_CameraSpace = new double[6];
      targetPose_RobotSpace = new double[6];
      fiducialID = -1;
  }
}


} // end class LImelight

