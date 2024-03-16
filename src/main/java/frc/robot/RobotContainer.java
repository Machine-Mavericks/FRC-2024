// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.GroundIntake;
import frc.robot.commands.GroundIntakeWithSensor;
import frc.robot.commands.OperatorSpinup;
import frc.robot.commands.RunClimbCommand;
import frc.robot.commands.ShootAmp;
import frc.robot.commands.UnstuckShot;
import frc.robot.commands.Autonomous.DelayCommand;
import frc.robot.commands.Autonomous.OneNoteAuto;
import frc.robot.commands.Autonomous.TwoNoteAuto;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.AimToSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.FinishIntake;
import frc.robot.commands.SemiAutonomous.SteerToNote;
import frc.robot.commands.SemiAutonomous.TurnRobot;
import frc.robot.commands.SemiAutonomous.TurnToSpeaker;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.subsystems.CassetteIntake;
import frc.robot.subsystems.CassetteShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDBlinkin;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NVidia;
import frc.robot.subsystems.NoteTargeting;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SpeakerTargeting;
//import frc.robot.subsystems.SwerveOdometry;
import frc.robot.subsystems.Odometry;

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
  public static final Limelight leftlimelight = new Limelight("left");
  public static final Limelight rightlimelight = new Limelight("right");
  public static final Limelight intakelimelight = new Limelight("intake");

  public static final NVidia nvidia = new NVidia();
  public static final Pigeon gyro = new Pigeon();
  public static final Drivetrain drivetrain = new Drivetrain();
  //public static final SwerveOdometry odometry = new SwerveOdometry();
  public static final Odometry odometry = new Odometry();
  //public static final PowerPanel panel = new PowerPanel();
  public static final LEDBlinkin LEDStrip = new LEDBlinkin();
  public static final CassetteShooter cassetteshooter = new CassetteShooter();
  public static final CassetteIntake cassetteintake = new CassetteIntake();
  public static final CassetteEffector cassetteangle = new CassetteEffector();
  public static final SpeakerTargeting speakertargeting = new SpeakerTargeting(shotlimelight);
  public static final NoteTargeting notetargeting = new NoteTargeting(intakelimelight);
  public static final Climber climber = new Climber();

  /**
   * Initialise the container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public static void init() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
    //LEDStrip.setDefaultCommand(new LEDCommand());

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
    OI.intakeButton.whileTrue(new GroundIntakeWithSensor(5));
    //OI.intakeButton.onFalse(new GroundIntake(false, 0.05));

    // Speaker shot
    OI.speakerShooterButton.whileTrue(new AimThenShootSpeaker());
    OI.speakerShooterButton.onFalse(new CleanupShot());
    
    // Amp shot
    OI.ampButton.onTrue(new ShootAmp());

    // Spit out notes
    OI.unstuckButton.whileTrue(new UnstuckShot());

    // Auto intake
    OI.autoIntakeButton.whileTrue(new SteerToNote(true, 3));
    //OI.autoIntakeButton.onFalse(new GroundIntake(false, 0.05)); 
    //OI.autoIntakeButton.onFalse(new FinishIntake());

    // Preemtively spin up shooter on command
    OI.spinupShooterButton.whileTrue(new OperatorSpinup());

    // Climb control
    OI.extendClimbButton.whileTrue(new RunClimbCommand(false));
    OI.retractClimbButton.whileTrue(new RunClimbCommand(true));

    // Spin 180
    OI.spinButton.onTrue(new TurnRobot(180.0, true, 2));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {  
    // get autonomous path to run
    int index = (Integer)RobotContainer.operatorinterface.m_autonomousPath.getSelected();
    Command chosenCommand;
    // return autonomous command to be run
    switch (index) {
      case 0:
        chosenCommand = new OneNoteAuto();
        break;
      case 1:
        chosenCommand = new TwoNoteAuto();
        break;
      default:
        chosenCommand = null;
        break;
    } 
    return new SequentialCommandGroup(
      new InstantCommand(()-> RobotContainer.cassetteangle.setAngle(CassetteEffector.DROP_PROP_ANGLE)), // Drop rickstand
      new DelayCommand(operatorinterface.getAutoDelay()), // Wait desired time before running chosen auto
      chosenCommand
    );
  }
}