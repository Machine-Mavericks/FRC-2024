// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StagePath extends SequentialCommandGroup {
  /** Creates a new StagePath. */
  public StagePath() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Move forwards 1 meter 
    //AimThenShootSpeaker 
    //SteerToNote 2
    //AimThenShootSpeaker 
    //Use April tag 15 to navigate stage 
    //SteerToNote 6 (pre load or shoot)

    //Use April tag 13 to navigate stage 
    //AimThenShootSpeaker 
    //End 

    addCommands();
  }
}
