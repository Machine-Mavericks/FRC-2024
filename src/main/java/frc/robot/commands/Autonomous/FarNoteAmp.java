// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FarNoteAmp extends SequentialCommandGroup {
  /** Creates a new FarNoteAmp. */
  public FarNoteAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Move Forwards 1 meter 
    //AimThenShootSpeaker 
    //SteerToNote 3
    //AimThenShootSpeaker 
    //SteerToNote 8
    //Move to far shooting position
    //AimThenShootSpeaker 
    //SteerToNote 7
    //Move to far shooting position 
    //End 
    
    addCommands();
  }
}
