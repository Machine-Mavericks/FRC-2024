// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RevUpShoot extends SequentialCommandGroup {
  /** Creates a new RevUpShoot. */
  public RevUpShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> RobotContainer.cassetteshooter.leftShootRun(RobotContainer.operatorInterface.LShooterSpeed.getDouble(0) / 60)),
      new InstantCommand(()-> RobotContainer.cassetteshooter.rightShootRun(RobotContainer.operatorInterface.RShooterSpeed.getDouble(0) / 60)),
      new DelayCommand(0.7),
      new InstantCommand(()-> RobotContainer.cassetteintake.intakeRun(1)),
      new InstantCommand(()-> RobotContainer.cassetteintake.intakeRun(1)),
      new DelayCommand(0.5),
      new InstantCommand(()-> RobotContainer.cassetteshooter.stopShooter()),
      new InstantCommand(()-> RobotContainer.cassetteintake.intakeRun(0))
    );
  }
}
