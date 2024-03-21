// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FarSource extends SequentialCommandGroup {
  /** Creates a new FarSource. */
  public FarSource() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //Move Forward 2 meeters
    //AimThenShootSpeaker 
    //SteerToNote 4
    //Move to previuse shoot position 
    //AimThenShootSpeaker 
    //SteerToNote 5 (preload or shoot based on time )
    //Move to previuse shoot position 
    //AimThenShootSpeaker 
    //End
  

    addCommands();
  }
}
