// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.DriveToRelativePose;
import frc.robot.commands.SetGyroUsingAprilTag;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.SteerToNote;
import frc.robot.subsystems.CassetteEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoDonutAuto extends SequentialCommandGroup {
  /** Creates a new OneDonutAuto. */
  public TwoDonutAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(


    // set gyro angle from odometry
    new SetGyroUsingAprilTag(),

    // drive away from speaker
    new DriveToRelativePose(new Pose2d(-1.0, 0.0, new Rotation2d(0)),
                            0.5, // speed
                            0.1, // rotational speed(unit?)
                            5.0),

    new AimThenShootSpeaker(),

      new InstantCommand(() -> RobotContainer.cassetteshooter.stopShooter()),
      new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE)),
    
    //new DelayCommand(0.5),

    new SteerToNote(true, 3.0),

    new AimThenShootSpeaker(),

    new CleanupShot()



    );
  }
}