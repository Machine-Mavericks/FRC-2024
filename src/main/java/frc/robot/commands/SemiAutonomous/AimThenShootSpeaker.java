// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.TurnToSpeaker;
import frc.robot.commands.Mechanism.ShootSpeaker;
import frc.robot.commands.Mechanism.SpinupSpeaker;
import frc.robot.commands.Mechanism.WaitForEffectorAngle;
import frc.robot.commands.Mechanism.WaitForShooterSpinup;


public class AimThenShootSpeaker extends SequentialCommandGroup {
  
  /** Creates a new AimThenShootSpeaker. */
  public AimThenShootSpeaker() {
    
    addCommands(  

      new SpinupSpeaker(),  
      new TurnToSpeaker(),
      new ParallelCommandGroup(
          new WaitForEffectorAngle(), 
          new WaitForShooterSpinup()
        ),
      new ShootSpeaker()
    );
  }
}
