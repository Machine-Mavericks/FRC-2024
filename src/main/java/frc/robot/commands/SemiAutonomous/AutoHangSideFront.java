// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoHangSideFront extends SequentialCommandGroup {
  /** Creates a new AutoHangSideFront. */
  public AutoHangSideFront() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (RobotContainer.odometry.getPose2d().getY()>=4.3 && RobotContainer.odometry.getPose2d().getX()<=5.6){
      addCommands(
       new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.75,5.10, new Rotation2d(Math.toRadians(120.0)))), 0.25, 0.5, 3)
      
      
      
      
       );
    }else if (RobotContainer.odometry.getPose2d().getY()<=3.7 && RobotContainer.odometry.getPose2d().getX()<=5.6){
      
    }{
      addCommands(
       new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.95,2.90, new Rotation2d(Math.toRadians(180.0)))), 0.25, 0.5, 3)
      
      
      
      
       );

    }
    
    
  }
}
