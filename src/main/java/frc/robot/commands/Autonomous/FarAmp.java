// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FarAmp extends SequentialCommandGroup {
  /** Creates a new FarAmp. */
  public FarAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Move forward 1 meter 
    //AimThenShootSpeaker 
    //Move inbetween note 2 and 3 
    //SteerToNote 8
    //Move to Far shooting position
    //AimThenShootSpeaker 
    //SteerToNote 7
    //Move to far shooting position 
    //AimThenShootSpeaker 
    //End 

    addCommands();
  }
}
