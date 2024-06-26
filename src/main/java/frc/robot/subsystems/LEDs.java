// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class LEDs extends SubsystemBase {
  
  // Addressable LED and buffer
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  
  private final int NumLEDs = 30;

  // used for simple light strobe
  private int StrobeIndex = 0;

  // timer to slow down strobe
  private int timer;
  private double counter;

  //PowerDistribution robot_pdp = new PowerDistribution();

  /** Creates a new LEDs. */
  public LEDs() {

    // create addressble LED controller
    m_led = new AddressableLED(RobotMap.PWMPorts.LED_STRIP);
    m_led.setLength(NumLEDs);
    
    // create LED buffer - length is # of LEDs in string
    m_ledBuffer = new AddressableLEDBuffer(NumLEDs);

    // Set the data
    //m_led.setLength (m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    
  }

  // This method will be called once per scheduler run
    @Override
  public void periodic() {
    
    timer++;
    if (timer>=4)
      timer=0;
    
    counter=counter+0.02;
    if (counter>1) 
        counter=0;

    if (timer==0)
    {
      StrobeIndex++;
      if (StrobeIndex>=NumLEDs) StrobeIndex=0;

      for (int i=0;i<NumLEDs; ++i)
          m_ledBuffer.setRGB(i, 0, 0, 0);

      //Note in intake change to orange
      if (RobotContainer.cassetteintake.NoteOrNoNote()==true){
        for (int i=0;i<NumLEDs;i=i+4)
        m_ledBuffer.setRGB(i, (int)(0*counter),(int)(255*counter), 0);
      }
      

      // // Update the LEDs to show that voltage is under 11V
      // double robot_voltage = robot_pdp.getVoltage();
      // if (robot_voltage <= 11.0){
      //   m_ledBuffer.setRGB(StrobeIndex, 137, 0, 204);
      // }      
    
      //when robot is in hang position change the LEDs to aqua?
      //idea for auto hang position? 
      //aqua RGB values are (r 62,g 254,b 255)

      if (DriverStation.getAlliance().get() == Alliance.Blue)
      {
        m_ledBuffer.setRGB(StrobeIndex, 0, 0, 255);
      }
      else
      {
        m_ledBuffer.setRGB(StrobeIndex, 255, 0, 0);
      }
    
      // If we see Notes go aqua
      if (StrobeIndex>=2 && RobotContainer.notetargeting.IsTarget() )
      for (int i=2;i<NumLEDs;i=i+4)
       m_ledBuffer.setRGB(i, (int)(133*counter),(int)(0*counter),(int)(191*counter));
      
      // if we are seeing Apriltags add Green
      if (StrobeIndex>=4 && RobotContainer.nvidia.GetNumberAprilTagsDetected()>=1)
        m_ledBuffer.setRGB(StrobeIndex-4, 255, 255, 255);

      

      m_led.setData(m_ledBuffer);
    }

  }
}