// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LEDBlinkin extends SubsystemBase {

  //Victor led;
  PWM led;
  /** Creates a new LEDBlinkin. */
  public LEDBlinkin() {

      // set up pwm channel
      //led = new Victor(RobotMap.PWMPorts.LED_BLINKIN);
      led = new PWM(RobotMap.PWMPorts.LED_BLINKIN);
      
      setPattern(LED_PATTERN.OFF);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public enum LED_PATTERN {
    OFF,
    LOWBATTERY,
    DISCO,
    NODE,
    READY,
    LOCKED,
    // Add
  };

  // sets pattern of LED strip
  public void setPattern(LED_PATTERN pattern)
  {
    switch (pattern) {
      case OFF:
        led.setSpeed(0.99);    // black
      break;
      case NODE:
        led.setSpeed(0.65);    // orange 
      break;
      case TARGET:
        led.setSpeed(0.57)     // pink
      break;
      case LOCKED:
        led.setSpeed(0.73)     // lime 
      break;
      case LOWBATTERY:
        led.setSpeed(0.91);    // solid purple
      break;
      case DISCO:
        led.setSpeed(-0.45);   // color wave - rainbow
      break;

    }
    
  }


}
