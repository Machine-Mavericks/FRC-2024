// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SemiAutonomous.AutoDriveToPose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SampleAutoCommand extends SequentialCommandGroup {
  /** Creates a new SampleAutoCommand2. */
  public SampleAutoCommand() {
    addCommands(
      //new AutoDriveToPose(AutoFunctions.NotesAtStart[3], Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 5)
      LastMinuteAutos(Wed)
    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(
    //     new FollowPath(TrajectoryConstants.TestPath.points,
    //     TrajectoryConstants.TestPath.startAngle,
    //         TrajectoryConstants.TestPath.endAngle,
    //         TrajectoryConstants.TestPath.startVelocity,
    //         TrajectoryConstants.TestPath.endVelocity,
    //         TrajectoryConstants.TestPath.endRobotAngle,
    //         TrajectoryConstants.TestPath.revserse,
    //         TrajectoryConstants.TestPath.rotatePath
    //         ) );
  }
}
