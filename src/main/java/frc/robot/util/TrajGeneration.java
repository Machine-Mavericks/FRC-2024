// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util; 
import java.util.List; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig; 
import edu.wpi.first.networktables.GenericEntry; 
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;   
import java.util.ArrayList; 
import java.util.Map;

public class TrajGeneration extends SubsystemBase {

    // Create a Shuffleboard tab and entries for the trajectory
    public ShuffleboardTab tab = Shuffleboard.getTab("TrajGen"); 
    private Trajectory trajectory;
    private GenericEntry  xPositionEntry;  
    private GenericEntry  yPositionEntry;
    private GenericEntry  velocityEntry; 
    private GenericEntry  accelerationEntry; 
    private GenericEntry  radEntry;

    // Define the swerve module positions relative to the robot's center.
    private  Translation2d kFrontLeftLocation = new Translation2d(0.35, 0.35);
    private  Translation2d kFrontRightLocation = new Translation2d(0.35, -0.35);
    private Translation2d kBackLeftLocation = new Translation2d(-0.35, 0.35);
    private Translation2d kBackRightLocation = new Translation2d(-0.35, -0.35);
    private SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            kFrontLeftLocation,
            kFrontRightLocation,
            kBackLeftLocation,
            kBackRightLocation ); 

    // Create constraint to ensure we don't accelerate too fast
    private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        0.30, // maxSpeedMetersPerSecond
        2.0 // maxAccelerationMetersPerSecondSquared
    ).setKinematics(kDriveKinematics);

  /** Creates a new TrajGeneration. */
  public TrajGeneration(double limitVel, double limitAcc) { 
    // initShuffleboard();
    trajectoryConfig = new TrajectoryConfig(limitVel, limitAcc).setKinematics(kDriveKinematics);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateShuffleboard();
  }

  public Trajectory genTraj(Pose2d starPose2d, Pose2d tarPose2d) {   
        trajectory =  TrajectoryGenerator.generateTrajectory(
            starPose2d,
            List.of(), // No interior waypoints
            tarPose2d,
            trajectoryConfig
        ); 
        return trajectory;
    }

    public Trajectory genTraj(Pose2d starPose2d, Pose2d tarPose2d, Translation2d wayPoint) {   
        
        trajectory =  TrajectoryGenerator.generateTrajectory(
            starPose2d,
            List.of(wayPoint), // No interior waypoints
            tarPose2d,
            trajectoryConfig
        ); 
        return trajectory;
    }

  public Trajectory genTrajThroughWaypoints(ArrayList<Translation2d> interiorWaypoints,Pose2d starPose2d, Pose2d tarPose2d) {
      trajectory = TrajectoryGenerator.generateTrajectory(starPose2d, interiorWaypoints, tarPose2d, trajectoryConfig);
      return trajectory;
  }

  public List<Pose2d> sampleTrajectory(Trajectory trajectory, double intervalSeconds) {
      List<Pose2d> sampledPoints = new ArrayList<>();
      for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += intervalSeconds) {
          Trajectory.State state = trajectory.sample(t);
          sampledPoints.add(state.poseMeters);
      }
      sampledPoints.add(trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters);
      return sampledPoints;
  } 
  
  public List<Double> samplePoseX(Trajectory trajectory, double intervalSeconds) {
      List<Double> sampledPointsX = new ArrayList<>();
      for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += intervalSeconds) {
          Trajectory.State state = trajectory.sample(t);
          sampledPointsX.add(state.poseMeters.getX());
      }
      sampledPointsX.add(trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getX());
      return sampledPointsX;
  }
  
  public List<Double> samplePoseY(Trajectory trajectory, double intervalSeconds) {
      List<Double> sampledPointsY = new ArrayList<>();
      for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += intervalSeconds) {
          Trajectory.State state = trajectory.sample(t);
          sampledPointsY.add(state.poseMeters.getY());
      }
      sampledPointsY.add(trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getY());
      return sampledPointsY;
  }

  public List<Double> sampleVel(Trajectory trajectory, double intervalSeconds) {
      List<Double> sampledPointsVel = new ArrayList<>();
      for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += intervalSeconds) {
          Trajectory.State state = trajectory.sample(t);
          sampledPointsVel.add(state.velocityMetersPerSecond);
      }
      sampledPointsVel.add(trajectory.getStates().get(trajectory.getStates().size() - 1).velocityMetersPerSecond);
      return sampledPointsVel;
  }
  
  public List<Double> sampleAcc(Trajectory trajectory, double intervalSeconds) {
      List<Double> sampledPointsAcc = new ArrayList<>();
      for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += intervalSeconds) {
          Trajectory.State state = trajectory.sample(t);
          sampledPointsAcc.add(state.accelerationMetersPerSecondSq);
      }
      sampledPointsAcc.add(trajectory.getStates().get(trajectory.getStates().size() - 1).accelerationMetersPerSecondSq);
      return sampledPointsAcc;
  }

  public List<Double> sampleRad(Trajectory trajectory, double intervalSeconds) {
      List<Double> sampledPointsRad = new ArrayList<>();
      for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += intervalSeconds) {
          Trajectory.State state = trajectory.sample(t);
          sampledPointsRad.add(state.poseMeters.getRotation().getRadians());
      }
      sampledPointsRad.add(trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation().getRadians());
      return sampledPointsRad;
  } 

  public Trajectory getTraj() {
    return trajectory;
  } 

  public double getTotalTimeSeconds() {
    return trajectory.getTotalTimeSeconds();
  }
  
  private void initShuffleboard() {  
    xPositionEntry = tab.add("Traj X ", new double[]{})
                        .withWidget(BuiltInWidgets.kGraph)
                        .withProperties(Map.of("Visible time", 30))
                        .getEntry();
    yPositionEntry = tab.add("Traj Y ", new double[]{})
                        .withWidget(BuiltInWidgets.kGraph)
                        .withProperties(Map.of("Visible time", 30))
                        .getEntry();
    velocityEntry = tab.add("Vel ", new double[]{})
                        .withWidget(BuiltInWidgets.kGraph)
                        .withProperties(Map.of("Visible time", 30))
                        .getEntry(); 
    accelerationEntry = tab.add("Acc ", new double[]{})
                        .withWidget(BuiltInWidgets.kGraph)
                        .withProperties(Map.of("Visible time", 30))
                        .getEntry();
    radEntry = tab.add("Rad ", new double[]{})
                        .withWidget(BuiltInWidgets.kGraph)
                        .withProperties(Map.of("Visible time", 30))
                        .getEntry(); 
  }

  public void updateShuffleboard() {
    List<Double> xPositionsList = samplePoseX(trajectory, 0.01);
    double[] xPositionsArray = xPositionsList.stream().mapToDouble(Double::doubleValue).toArray();
    xPositionEntry.setDoubleArray(xPositionsArray); 

    List<Double> yPositionsList = samplePoseY(trajectory, 0.01);
    double[] yPositionsArray = yPositionsList.stream().mapToDouble(Double::doubleValue).toArray();
    yPositionEntry.setDoubleArray(yPositionsArray);

    List<Double> velList = sampleVel(trajectory, 0.01);
    double[] velArray = velList.stream().mapToDouble(Double::doubleValue).toArray(); 
    velocityEntry.setDoubleArray(velArray);
     
    List<Double> accList = sampleAcc(trajectory, 0.01);
    double[] accArray = accList.stream().mapToDouble(Double::doubleValue).toArray();
    accelerationEntry.setDoubleArray(accArray); 

    List<Double> radList = sampleRad(trajectory, 0.01);
    double[] radArray = radList.stream().mapToDouble(Double::doubleValue).toArray();
    radEntry.setDoubleArray(radArray);
  }

}
