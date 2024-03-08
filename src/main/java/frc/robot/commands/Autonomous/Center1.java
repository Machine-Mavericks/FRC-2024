// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeMoveToHoldingPosition;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.SteerToNote;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Center1 extends SequentialCommandGroup {
  /** Creates a new Center1. */
  public Center1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    //Move forwards a meter
    //AimThenShootSpeaker 
    //SteerToNote 0
    //ShootSpeaker 
    //End
    addCommands(
      // set our robot starting position in field coordinates
      //new DriveToRelativePose(new Pose2d(1,0,new Rotation2d(3.14))),
      new AimThenShootSpeaker(),
      new SteerToNote(true,2),
      new IntakeMoveToHoldingPosition(),
      new AimThenShootSpeaker()





    );
  }
}
