// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CleanupShot extends SequentialCommandGroup {
  /** Creates a new CleanupShot. */
  public CleanupShot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> RobotContainer.operatorinterface.ShooterAtAngle.setBoolean(false)),
      new InstantCommand(() -> RobotContainer.operatorinterface.ShooterAtSpeed.setBoolean(false)),
      new InstantCommand(() -> RobotContainer.operatorinterface.RobotAtAngle.setBoolean(false)),
      new InstantCommand(() -> RobotContainer.operatorinterface.TargetDistance.setDouble(0)),
      new InstantCommand(() -> RobotContainer.cassetteshooter.stopShooter()),
      new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE)),
      new IntakeMoveToHoldingPosition()
    );
  }
}
