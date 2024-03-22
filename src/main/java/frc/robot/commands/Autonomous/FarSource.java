// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeMoveToHoldingPosition;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.AutoDriveToPose;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.SteerToNote;
import frc.robot.commands.SemiAutonomous.TurnRobot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;

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
    
   

    addCommands(
    
    // new TurnRobot(360, true, 3),

    new AimThenShootSpeaker(),

    new CleanupShot(),

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  0.7,7.5, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

    new SteerToNote(true, 2.0, 0.2),

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2,4, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    new AimThenShootSpeaker(),

    new CleanupShot(),

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
    
    new SteerToNote(true, 2.0, 0.2),

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    

    new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2,4, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    new AimThenShootSpeaker(),

    new CleanupShot()

    );
  }
}
