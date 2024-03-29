// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.Autonomous.DelayCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimThenShootSpeaker extends SequentialCommandGroup {
  /** Creates a new AimThenShootSpeaker. */
  public AimThenShootSpeaker() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new ParallelRaceGroup(
      new SpinupSpeaker(),  
      new TurnToSpeaker()
    ),
    
    new ParallelCommandGroup(
       new WaitForEffectorAngle(), 
       new WaitForShooterSpinup()
     ),
    new ShootSpeaker()
    
    
    // new ParallelRaceGroup(
      //   new AimToSpeaker(),
      //   )
      // ),
      // new ParallelCommandGroup(
      //   new WaitForEffectorAngle(), 
      //   new WaitForShooterSpinup()
      // ),
      // new DelayCommand(2.0),
      // new ShootSpeaker()
    );
  }
}
