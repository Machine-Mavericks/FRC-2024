// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.commands.Mechanism.SpinupSpeaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveThenShoot extends SequentialCommandGroup {
  /**
   * Creates a new MoveThenShoot
   * @param target target position on field
   * @param wayPoint set to null if no waypoint
   * @param speed
   * @param rotationalspeed
   * @param timeout
   */
  public MoveThenShoot(Pose2d target, Translation2d wayPoint, double speed, double rotationalspeed, double timeout) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        // spinup
        new SpinupSpeaker(),
        // cassette angle
        new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(RobotContainer.speakertargeting.getDesiredAngle(target))),
        // auto drive to pose with angle away from speaker
        new AutoDriveToFieldPose(new Pose2d(target.getX(),target.getY(),new Rotation2d(RobotContainer.speakertargeting.getSpeakerAngle(target))),
                                wayPoint,speed,rotationalspeed,timeout)
      )
    );
  }
}
