// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class LEDBlinkin extends SubsystemBase {
  //Victor led;
  Spark led;

  /** Creates a new LEDBlinkin. */
  public LEDBlinkin() {
    // set up pwm channel
    //led = new Victor(RobotMap.PWMPorts.LED_BLINKIN);
    led = new Spark(RobotMap.PWMPorts.LED_BLINKIN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (RobotContainer.speakertargeting.IsTarget()) {
     RobotContainer.LEDStrip.setPattern(LED_PATTERN.APRILTAGS);
    } 
    else if (RobotContainer.notetargeting.IsTarget()) {
      RobotContainer.LEDStrip.setPattern(LED_PATTERN.SEESNOTES);
    }
    else if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        RobotContainer.LEDStrip.setPattern(LED_PATTERN.REDALLIANCE);
      } else {
      RobotContainer.LEDStrip.setPattern(LED_PATTERN.BLUEALLIANCE);  
      }
    } else {
      RobotContainer.LEDStrip.setPattern(LED_PATTERN.BLUEALLIANCE);  
    }
  }

  public enum LED_PATTERN {
    OFF,
    LOWBATTERY,
    DISCO,
    REDALLIANCE,
    BLUEALLIANCE,
    SEESNOTES,
    APRILTAGS
    // Add
  };

  // sets pattern of LED strip
  public void setPattern(LED_PATTERN pattern)
  {
    switch (pattern) {
      
      case OFF:
        led.set(0.99);    // black
        break;
      case LOWBATTERY:
        led.set(0.91);    // solid purple
        break;
      case DISCO:
        led.set(-0.45);   // color wave - rainbow
        break;
      case REDALLIANCE:
        led.set(0.360); // red 
        break;
      case BLUEALLIANCE:
        led.set(0.240); // blue 
        break; 
      case SEESNOTES:
        led.set(-0.120); // pink
        break;
      case APRILTAGS:
        led.set(0.120); //green
        break;
      default:
        led.set(0.99);    // black
        break;
    }
  }


}
