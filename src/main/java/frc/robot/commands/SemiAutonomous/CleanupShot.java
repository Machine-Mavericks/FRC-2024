// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteEffector;


public class CleanupShot extends SequentialCommandGroup {
  
  /** Creates a new CleanupShot. */
  public CleanupShot() {
    
    addCommands(
      new InstantCommand(() -> RobotContainer.operatorinterface.ShooterAtAngle.setBoolean(false)),
      new InstantCommand(() -> RobotContainer.operatorinterface.ShooterAtSpeed.setBoolean(false)),
      new InstantCommand(() -> RobotContainer.cassetteshooter.stopShooter()),
      new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE)),
      new InstantCommand(() -> RobotContainer.cassetteintake.intakeRun(0.0)),
      
      new PrintCommand("intake at holding position")
    );
  }
}
