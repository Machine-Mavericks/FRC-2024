// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.NoteTargeting;
import frc.robot.subsystems.LEDBlinkin.LED_PATTERN;

/** Command controls LEDs to display required information to driver */
public class LEDCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  /**
   * Creates a new LEDCommand.
   */
  public LEDCommand() {
    //addRequirements(RobotContainer.LEDStrip);
    addRequirements(RobotContainer.LEDStrip);
   
  
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  int counter = 0;
  @Override
  public void execute() {
    
    counter +=1;
    if (counter>25)
      counter=0;
     
    // insert if statements to change LED patterns
    
      
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0); // Turn off LEDs when command ends.
    RobotContainer.LEDStrip.setPattern(LED_PATTERN.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
