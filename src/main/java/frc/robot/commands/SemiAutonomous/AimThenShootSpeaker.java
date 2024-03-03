// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootSpeaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimThenShootSpeaker extends SequentialCommandGroup {
  /** Creates a new AimThenShootSpeaker. */
  public AimThenShootSpeaker() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (!RobotContainer.speakertargeting.IsTarget()) {
      System.out.println("Nope.");
      return;
    }

    addCommands(
      new InstantCommand(() -> RobotContainer.shotlimelight.setPipeline(1)),
      new ParallelRaceGroup(
        new AimToSpeaker(),
        new SpinupSpeaker()
      ),
      new ParallelCommandGroup(
        new WaitForEffectorAngle(), 
        new WaitForShooterSpinup()
      ),
      new ShootSpeaker()
    );
  }
}
