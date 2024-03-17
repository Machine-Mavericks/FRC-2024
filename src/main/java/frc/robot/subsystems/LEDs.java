package frc.robot.subsystems;

import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class LEDs extends SubsystemBase{
  
  // Addressable LED and buffer
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  
  private final int NumLEDs = 30;

  // used for simple light strobe
  private int StrobeIndex = 0;

  // timer to slow down strobe
  private int timer;
  private double counter;

 // Reach into the PowerDistribution module to get the voltage, etc.
  PowerDistribution robot_pdp = new PowerDistribution();
  double robot_voltage = robot_pdp.getVoltage();
  
  /** Creates a new LEDs. */
  public LEDs() {
    // create addressble LED controller
    m_led = new AddressableLED(RobotMap.PWMPorts.LEDs);
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
  public void periodic(){
    
    timer++;
    if (timer>=4)
      timer=0;
    
    counter=counter+0.02;
    if (counter>1) 
       counter=0;

    if (timer==0)
    {
      StrobeIndex++;
      if (StrobeIndex>=NumLEDs)
        StrobeIndex=0;

         // LEDs set to off  
        for (int i=0;i<NumLEDs; ++i)
          m_ledBuffer.setRGB(i, 0, 0, 0);
      
        //Note in intake change to orenge 
      if (RobotContainer.cassetteintake.NoteOrNoNote()==true){
        for (int i=0;i<NumLEDs;i=i+2)
        m_ledBuffer.setRGB(i, (int)(255.0*counter),(int)(50.0*counter), 0);
      }

      //note detection going pink 
      // if (RobotContainer.notetargeting.IsTarget()==true){
      //  for (int i=0;i<NumLEDs;i=i+2)
      //   m_ledBuffer.setRGB(i, 255*counter,123*counter, 215*counter);
      // }
      robot_voltage = robot_pdp.getVoltage();
      // Update the LEDs to show that voltage is under 11V
      if (robot_voltage <= 11.0){
        m_ledBuffer.setRGB(StrobeIndex, 137, 0, 204);
      }      
    

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
      if (StrobeIndex>=2 && RobotContainer.nvidia.GetNumberAprilTagsDetected()>=1)
        m_ledBuffer.setRGB(StrobeIndex-2, 0, 255, 0);
        
      m_led.setData(m_ledBuffer);
    }
  }
}