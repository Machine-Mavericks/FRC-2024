// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SemiAutonomous.AutoDriveToPose;
import frc.robot.commands.SemiAutonomous.TurnRobot;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicErAuto extends SequentialCommandGroup {
  /** Creates a new BasicErAuto. */
  public BasicErAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new TurnRobot(360, true, 3),
      new AutoDriveToPose(new Pose2d(0,0,new Rotation2d(180)), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3)
    );
  }
}
