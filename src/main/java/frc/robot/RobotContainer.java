// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.IntakeMoveToHoldingPosition;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.OldShootSpeaker;
import frc.robot.commands.OperatorSpinup;
import frc.robot.commands.SourceIntake;
import frc.robot.commands.UnstuckShot;
import frc.robot.commands.Autonomous.SampleAutoCommand;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.AutoDriveToPose;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.SteerToNote;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.subsystems.CassetteIntake;
import frc.robot.subsystems.CassetteShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDBlinkin;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NVidia;
import frc.robot.subsystems.NoteTargeting;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SpeakerTargeting;
//import frc.robot.subsystems.SwerveOdometry;
import frc.robot.subsystems.SwervePoseEstimator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** Update loop rate in Hz */
  public static final double updateDt = 0.02;

  // Create robot's shuffboard operator interface
  public static final ShuffleboardOI operatorinterface = new ShuffleboardOI();

  // The robot's subsystems are defined here...
  //public static final Gyro gyro = new Gyro();
  public static final Limelight shotlimelight = new Limelight("shoot");
  public static final Limelight intakelimelight = new Limelight("intake");

  //public static final NVidia nvidia = new NVidia();
  public static final Pigeon gyro = new Pigeon();
  public static final Drivetrain drivetrain = new Drivetrain();
  //public static final SwerveOdometry odometry = new SwerveOdometry();
  public static final SwervePoseEstimator swervepose = new SwervePoseEstimator();
  //public static final PowerPanel panel = new PowerPanel();
  public static final LEDBlinkin LEDStrip = new LEDBlinkin();
  public static final CassetteShooter cassetteshooter = new CassetteShooter();
  public static final CassetteIntake cassetteintake = new CassetteIntake();
  public static final CassetteEffector cassetteangle = new CassetteEffector();
  public static final SpeakerTargeting speakertargeting = new SpeakerTargeting(shotlimelight);
  public static final NoteTargeting notetargeting = new NoteTargeting(intakelimelight);

  /**
   * Initialise the container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public static void init() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
    LEDStrip.setDefaultCommand(new LEDCommand());

    // Camera Servers:
    //CameraServer.	startAutomaticCapture(0);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {
    // Zero gyro for driving
    OI.zeroButton.whileTrue(new RunCommand(() -> gyro.resetGyro()));

    // Manual intake
    OI.intakeButton.whileTrue(new GroundIntake(true));
    OI.intakeButton.onFalse(new IntakeMoveToHoldingPosition());

    // Speaker shot
    OI.speakerShooterButton.whileTrue(new AimThenShootSpeaker());
    OI.speakerShooterButton.onFalse(new CleanupShot());
    
    // Amp shot
    OI.ampButton.onTrue(new ShootAmp());

    // Spit out notes
    OI.unstuckButton.whileTrue(new UnstuckShot());
    OI.autoIntakeButton.whileTrue(new SteerToNote(true, 3));

    // Preemtively spin up shooter on command
    OI.spinupShooterButton.whileTrue(new OperatorSpinup());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
   return new SampleAutoCommand(); //filler, replace with autonomous path
  }
}