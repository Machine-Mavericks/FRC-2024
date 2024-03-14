// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CassetteIntake;
import frc.robot.subsystems.SpeakerTargeting;

public class FireShot extends Command {
  private static final double SHOT_TIMEOUT = 1;

  private final CassetteIntake intake;
  private final SpeakerTargeting speakerTargeting;

  private boolean valid = false;

  private Timer timer;

  /** Creates a new FireShot. */
  public FireShot() {
    intake = RobotContainer.cassetteintake;
    speakerTargeting = RobotContainer.speakertargeting;

    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    valid = speakerTargeting.IsSpunUp() && speakerTargeting.IsAligned();
    if (valid) {
      intake.intakeRun(1);
      timer.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeRun(0);
  }

  // Make sure shot command can't be canceled
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: add sensor condition to end early once sensor is installed
    return (!valid) || timer.hasElapsed(SHOT_TIMEOUT);
  }
}
