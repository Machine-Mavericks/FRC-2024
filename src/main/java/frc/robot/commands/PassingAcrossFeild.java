// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SemiAutonomous.SpinupSpeaker;
import frc.robot.commands.SemiAutonomous.TurnToSpeaker;
import frc.robot.commands.SemiAutonomous.WaitForEffectorAngle;
import frc.robot.commands.SemiAutonomous.WaitForShooterSpinup;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.subsystems.CassetteShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PassingAcrossFeild extends SequentialCommandGroup {
  /** Creates a new PassingAcrossFeild. */
  public PassingAcrossFeild() {
    addRequirements(RobotContainer.cassetteshooter);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands (
      new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.GROUND_ANGLE)),
      new InstantCommand(() -> RobotContainer.cassetteshooter.leftShootRun(3000.0)),
      new InstantCommand(() -> RobotContainer.cassetteshooter.rightShootRun(5000.0)),
      
    // Get to speed

    new ParallelCommandGroup(
       new WaitForEffectorAngle(), 
       new WaitForShooterSpinup()
     ),
    new ShootSpeaker()
    );
  }
}
