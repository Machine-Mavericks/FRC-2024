// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.AutoDriveToPose;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.DriveToRelativePose;
import frc.robot.commands.SemiAutonomous.SteerToNote;
import frc.robot.commands.SemiAutonomous.TurnRobot;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FarAmp extends SequentialCommandGroup {
  /** Creates a new FarAmp. */

  public FarAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // set gyro angle from odometry
      //new SetGyroUsingAprilTag(),
      // new TurnRobot(360, true, 3),
      //move to note 2
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      new AimThenShootSpeaker(),

      new CleanupShot(),
      // square up
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
      // pick up note 2
      new SteerToNote(true, 2.0, 0.2),
      
      // square up
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
      
      new AimThenShootSpeaker(),

      new CleanupShot(),
      // square up 
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,6.9,new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      // move to note 3
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(7.3,7.5, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      new SteerToNote(true, 2.0, 0.2),

      // square up
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(7.3,7.5, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,6.9,new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      new AimThenShootSpeaker(),

      new CleanupShot(),
      // move to note 4 
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(7.5,5.8,new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
      // square up
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
      
      new SteerToNote(true, 2.0, 0.2),
      // square up 
      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(7.5,5.8,new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

      new AimThenShootSpeaker(),

      new CleanupShot()

    );
  }

}

