// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.commands.Mechanism.GroundIntake;
import frc.robot.commands.Mechanism.WaitForEffectorAngle;
import frc.robot.commands.Mechanism.WaitForShooterSpinup;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.subsystems.CassetteIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteSource extends SequentialCommandGroup {
  /** Creates a new ThreeNoteSource. */
  public ThreeNoteSource() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //Shoot preload 
    new AimThenShootSpeaker(), 
    new CleanupShot(),
    //new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.5,1.3, new Rotation2d(Math.toRadians(180.0)))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
    
    new ParallelCommandGroup( 
          new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(8.4,0.8, new Rotation2d(Math.toRadians(-150.0)))), 0.5, 0.5, 3),
          new GroundIntake(5)
    ),
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(Math.toRadians(180.0)))), 0.5,0.5, 3),
    new AimThenShootSpeaker(),
    new CleanupShot()

    );
  }
}
