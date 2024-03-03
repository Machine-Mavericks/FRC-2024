// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OldShootSpeaker extends SequentialCommandGroup {
  /** Creates a new ShootSpeaker. */
  public OldShootSpeaker() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> RobotContainer.cassetteangle.setAngle(RobotContainer.operatorinterface.EffectorTarget.getDouble(0.05))),
      new InstantCommand(()-> RobotContainer.cassetteshooter.leftShootRun(RobotContainer.operatorinterface.LShooterSpeed.getDouble(0))),
      new InstantCommand(()-> RobotContainer.cassetteshooter.rightShootRun(RobotContainer.operatorinterface.RShooterSpeed.getDouble(0))),
      // Commands.parallel(new WaitForEffectorAngle(), new WaitForShooterSpinup()),
      new ParallelCommandGroup(
        new WaitForEffectorAngle(), 
        new WaitForShooterSpinup()
      ),
      new InstantCommand(()-> RobotContainer.cassetteintake.intakeRun(1)),
      new DelayCommand(0.5),
      new InstantCommand(()-> RobotContainer.cassetteshooter.stopShooter()),
      new InstantCommand(()-> RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE)),
      new InstantCommand(()-> RobotContainer.cassetteintake.intakeRun(0)),
      new CleanupShot()
    );
  }
}