// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.rtf.RTFEditorKit;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.ShuffleUser;
import frc.robot.util.SubsystemShuffleboardManager;

public class DistanceSensors extends SubsystemBase implements ShuffleUser{

  private SerialPort m_Adafruit;
  private boolean ConnectionSuccessful = false;

  private GenericEntry ReadString;
  private GenericEntry DataValidEntry;
  private GenericEntry Sensor0DistanceEntry;
  private GenericEntry Sensor1DistanceEntry;
  private GenericEntry MissedSamplesEntry;

  private LinearFilter Sensor0Filter = LinearFilter.singlePoleIIR(RobotContainer.updateDt * FILTER_PERIOD_FACTOR, RobotContainer.updateDt);
  private LinearFilter Sensor1Filter = LinearFilter.singlePoleIIR(RobotContainer.updateDt * FILTER_PERIOD_FACTOR, RobotContainer.updateDt);

  private static final int FILTER_PERIOD_FACTOR = 4;

  private double Sensor0Distance = 0;
  private double Sensor1Distance = 0;

  private String RawString;
  private String LastString;
  private boolean DataValid;

  private int MissedSamples = 0;

  private Port[] Ports = {Port.kUSB, Port.kUSB1, Port.kUSB2};

  
  /** Creates a new DistanceSensors. */
  public DistanceSensors() {
    TryConnectSensor();
    SubsystemShuffleboardManager.RegisterShuffleUser(this, true, 20);
  }

  private void TryConnectSensor(){
    MissedSamples = 0;
    for (Port port : Ports) {
      boolean failed = false;
      try {
        m_Adafruit = new SerialPort(115200, port);
      }catch (Exception e){
        failed = true;
      }

      if (!failed) {
        ConnectionSuccessful = true;
        System.out.println("Connected to adafruit distance sensor on port " + port.toString());
        break;
      }
    }

    if (!ConnectionSuccessful) {
      System.out.println("Failed to connect to adafruit distance sensor!");
    }
  }

  @Override
  public void periodic() {
    try{
      updateReadings();
    }catch (Exception e){      
      RawString = "";
      DataValid = false;
    }

    if (LastString == RawString) {
      MissedSamples++;
    }else{
      MissedSamples = 0;
    }

    if (MissedSamples > 50) {
      resetSensor();
    }

    LastString = RawString;
  }

  private void resetSensor(){
    System.out.println("Trying to reset distance sensor...");

    MissedSamples = 0;

    if (m_Adafruit != null) {
      m_Adafruit.close();
      m_Adafruit = null;
    }
    
    DataValid = false;
    Sensor0Distance = 0;
    Sensor1Distance = 0;
    TryConnectSensor();
  }

  private void updateReadings(){
    if (m_Adafruit.getBytesReceived() > 0){
      RawString = m_Adafruit.readString();

      // Refuse to care about errors since it's all try() catched anyways
      String[] strings = RawString.split("\\s+"); // Ew it's regex
      if (!(strings.length >= 2)) {
        return;
      }
      
      Sensor0Distance = Integer.parseInt(strings[0]);
      Sensor1Distance = Integer.parseInt(strings[1]);

      // Convert mm to cm
      Sensor0Distance /= 10;
      Sensor1Distance /= 10;

      DataValid = true;
    }
  }

  public double getSensor0(){
    return Sensor0Distance;
  }

  public double getSensor1(){
    return Sensor1Distance;
  }

  @Override
  public void initializeShuffleboard() {
    ShuffleboardTab Tab = Shuffleboard.getTab("Distance Sensors");
    ReadString = Tab.add("Raw string", "").withPosition(0, 0).getEntry();
    DataValidEntry = Tab.add("Data Valid", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 1)
        .getEntry();

    Sensor0DistanceEntry = Tab.add("Sensor 0 Value", 0).withPosition(1, 0).getEntry();
    Sensor1DistanceEntry = Tab.add("Sensor 1 Value", 0).withPosition(1, 1).getEntry();

    MissedSamplesEntry = Tab.add("Missed Samples", 0).withPosition(2, 0).getEntry();
  }

  @Override
  public void updateShuffleboard() {
    ReadString.setString(RawString);
    DataValidEntry.setBoolean(DataValid);

    Sensor0DistanceEntry.setDouble(Sensor0Distance);
    Sensor1DistanceEntry.setDouble(Sensor1Distance);
    
    MissedSamplesEntry.setDouble(MissedSamples);
  }
}
