// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
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
      new InstantCommand(()-> RobotContainer.swervepose.setPosition(5.0, 2.0, 0.0, 0.0)),

      // move robot to x,y,angle
      // todo >

      // shoot into speaker
      new AimThenShootSpeaker(),
      //new AutoDriveToPose(AutoFunctions.redVsBlue(AutoFunctions.NotesAtStart[0]), Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 5)
      new SteerToNote(true,2)





    );
  }
}
