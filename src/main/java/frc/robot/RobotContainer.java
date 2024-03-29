// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.FiveNoteAmp;
import frc.robot.commands.Autonomous.FourNoteSource;
import frc.robot.commands.Autonomous.OneNoteAnywhere;
import frc.robot.commands.Autonomous.SixNoteAmp;
import frc.robot.commands.Autonomous.ThreeNoteStage;
import frc.robot.commands.Autonomous.TwoNoteAmp;
import frc.robot.commands.Autonomous.TwoNoteCenter;
import frc.robot.commands.Drive.ManualDriveCommand;
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.Drive.TurnToSpeaker;
import frc.robot.commands.Mechanism.GroundIntake;
import frc.robot.commands.Mechanism.OperatorSpinup;
import frc.robot.commands.Mechanism.RunClimbCommand;
import frc.robot.commands.Mechanism.UnstuckShot;
import frc.robot.commands.Other.DelayCommand;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.PassingAcrossField;
import frc.robot.subsystems.CassetteEffector;
import frc.robot.subsystems.CassetteIntake;
import frc.robot.subsystems.CassetteShooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NVidia;
import frc.robot.subsystems.NoteTargeting;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.SpeakerTargeting;


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
  public static final Limelight intakelimelight = new Limelight("intake");
  public static final NVidia nvidia = new NVidia();
  public static final Pigeon gyro = new Pigeon();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Odometry odometry = new Odometry();
  //public static final PowerPanel panel = new PowerPanel();
  public static final LEDs leds = new LEDs();
  public static final CassetteShooter cassetteshooter = new CassetteShooter();
  public static final CassetteIntake cassetteintake = new CassetteIntake();
  public static final CassetteEffector cassetteangle = new CassetteEffector();
  public static final SpeakerTargeting speakertargeting = new SpeakerTargeting();
  public static final Climber climber = new Climber();
  public static final NoteTargeting notetargeting = new NoteTargeting(intakelimelight);

  /**
   * Initialise the container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public static void init() {
    drivetrain.setDefaultCommand(new ManualDriveCommand(drivetrain));

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
    OI.intakeButton.whileTrue(new GroundIntake(5));
    //OI.intakeButton.onFalse(new GroundIntake(false, 0.05));

    // Speaker shot
    OI.speakerShooterButton.whileTrue(new AimThenShootSpeaker());
    OI.speakerShooterButton.onFalse(new CleanupShot());
    
    // Amp shot
    //.ampButton.onTrue(new ShootAmp());

    // Spit out notes
    OI.unstuckButton.whileTrue(new UnstuckShot());

    // Auto intake
    OI.autoIntakeButton.whileTrue(new SteerToNote(true, 3));

    // Preemtively spin up shooter on command
    OI.spinupShooterButton.whileTrue(new OperatorSpinup());

    // Passing acress the feild buttion 
    OI.passingAcrossFieldButton.whileTrue(new PassingAcrossField());
    OI.passingAcrossFieldButton.onFalse(new CleanupShot());
    // Climb control
    OI.extendClimbButton.whileTrue(new RunClimbCommand(false));
    OI.retractClimbButton.whileTrue(new RunClimbCommand(true));

    // blindly turn on intake to shoot
    OI.advanceIntakeButton.whileTrue(new InstantCommand(()-> RobotContainer.cassetteintake.intakeRun(1.0)));
    OI.advanceIntakeButton.onFalse(new InstantCommand(()-> RobotContainer.cassetteintake.intakeRun(0.0)));
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
        chosenCommand = new OneNoteAnywhere();
        break;
      case 1:
        chosenCommand = new TwoNoteAmp();
        break;
      case 2:
        chosenCommand = new FiveNoteAmp();
        break;
      case 3:
        chosenCommand = new FourNoteSource();
        break;
      case 4:
        chosenCommand = new DelayCommand(20); // Do nothing auto
        break;
      case 5:
        chosenCommand = new SixNoteAmp();
        break;
      case 6:
        chosenCommand = new TwoNoteCenter();
        break;
      case 7:
        chosenCommand = new ThreeNoteStage();
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