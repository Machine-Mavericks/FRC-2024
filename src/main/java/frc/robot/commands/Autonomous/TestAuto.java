// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.util.AutoFunctions;


public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.0 ,6.0,new Rotation2d(Math.toRadians(-90.0)))),  4.0, 0.15, 40)


    );
  }
}
