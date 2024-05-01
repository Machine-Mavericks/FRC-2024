// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Mechanism.ShootSpeaker;
import frc.robot.commands.Mechanism.WaitForEffectorAngle;
import frc.robot.commands.Mechanism.WaitForShooterSpinup;
import frc.robot.subsystems.CassetteEffector;


public class PassingAcrossField extends SequentialCommandGroup {
  
  /** Creates a new PassingAcrossFeild. */
  public PassingAcrossField() {
    
    addRequirements(RobotContainer.cassetteshooter, RobotContainer.cassetteangle);
    
    addCommands (
      new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.PASSING_ANGLE)),
      new InstantCommand(() -> RobotContainer.cassetteshooter.leftShootRun(2000.0)),
      new InstantCommand(() -> RobotContainer.cassetteshooter.rightShootRun(4000.0)),
      
    // Get to speed

    new ParallelCommandGroup(
       new WaitForEffectorAngle(), 
       new WaitForShooterSpinup()
     ),

    new ShootSpeaker(), 
    new WaitCommand(1),
    new CleanupShot()
    );
  }
}