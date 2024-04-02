// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveOverLine;
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.Drive.TurnRobot;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoNoteAnywhere extends SequentialCommandGroup {
  /** Creates a new TwoNoteEnywhere. */
  public TwoNoteAnywhere() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //new InstantCommand (()->RobotContainer.gyro.resetGyroReverse()),
    // Shoot Pre Load 
    new AimThenShootSpeaker(),
    new CleanupShot(),
    // Turn Back to Straight
    new TurnRobot(AutoFunctions.redVsBlue(180.0), false, 1.0),
    // Find Another Note 
    new SteerToNote(true, 3.0, 0.2),
    // Shoot 
    new AimThenShootSpeaker(),
    new CleanupShot()
   
    );
  }
}
