// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
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
    System.out.println("Setup Blinkin on port " + RobotMap.PWMPorts.LED_BLINKIN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (RobotContainer.speakertargeting.IsTarget()) {
     setPattern(LED_PATTERN.OFF); // Once targeting changes are made, change this to SHOTREADY
    } 
    else if (RobotContainer.notetargeting.IsTarget()) {
      setPattern(LED_PATTERN.SEESNOTES);
    }
    else if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        setPattern(LED_PATTERN.REDALLIANCE);
      } else {
      setPattern(LED_PATTERN.BLUEALLIANCE);  
      }
    } else {
      setPattern(LED_PATTERN.BLUEALLIANCE);  
    }
  }

  public enum LED_PATTERN {
    OFF,
    LOWBATTERY,
    DISCO,
    REDALLIANCE,
    BLUEALLIANCE,
    SEESNOTES,
    SHOTREADY
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
        led.set(-0.99);   // color wave - rainbow
        break;
      case REDALLIANCE:
        led.set(0.61); // red 
        break;
      case BLUEALLIANCE:
        led.set(0.87); // blue 
        break; 
      case SEESNOTES:
        led.set(-0.05); // gold strobe
        break;
      case SHOTREADY:
        led.set(0.73); // green
        break;
      default:
        led.set(0.99);    // black
        break;
    }
  }
}