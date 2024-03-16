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
      
      if (RobotContainer.cassetteintake.NoteOrNoNote()==true){
       for (int i=0;i<NumLEDs;i=i+2)
          m_ledBuffer.setRGB(i, 255*counter,50*counter, 0);

      }
      //note detection 
      // if (RobotContainer.notetargeting.IsTarget()==true){
      //  for (int i=0;i<NumLEDs;i=i+2)
      //   m_ledBuffer.setRGB(i, 255*counter,51, 204*counter);
      }

      
     
    
      if (DriverStation.getAlliance().get() == Alliance.Blue)
      {
        m_ledBuffer.setRGB(StrobeIndex, 0, 0, 255);
      }
      else
      {
        m_ledBuffer.setRGB(StrobeIndex, 255, 0, 0);
      }
      
        

      m_led.setData(m_ledBuffer);
    }

  }
}