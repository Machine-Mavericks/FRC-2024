// Shuffleboard
//
// Initializes and updates shuffleboard
// This module contains code for making and maintaining main shuffleboard page
// Other pages made by the individual subsystems as req'd

package frc.robot;


import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.BuildConstants;
import frc.robot.subsystems.CassetteEffector;


/** Contains shuffleboard setup for generic main page not belonging to any subsubsystem
 * Typically used to have robot-level settings and command functions */
public class ShuffleboardOI extends SubsystemBase {

    // example autonomous path shuffleboard selection boxes
    // true if selected, false if not
    // <add any other controls here that go on main shufflebard page
    private GenericEntry m_delayTime;
    public SendableChooser<Integer> m_autonomousPath;

    
    public GenericEntry EffectorTarget;
    public GenericEntry ShotCharacterizationTargetR;
    public GenericEntry ShotCharacterizationTargetL;

    // Shot info
    public GenericEntry LShooterSpeed;
    public GenericEntry RShooterSpeed;

    public GenericEntry SeesTarget;
    public GenericEntry ShooterAtSpeed;
    public GenericEntry ShooterAtAngle;
    public GenericEntry RobotAtAngle;
    public GenericEntry TargetDistance;
    public GenericEntry tY;

    public GenericEntry DistanceAdjustment;

    // other controls on main page
    private GenericEntry m_timeLeft;
    public Integer m_selectedPath;

    /** Initializes the Shuffleboard
     * Creates the subsystem pages */
    public ShuffleboardOI() {

        // add autonomous commands to shuffleboard
        initializeMainShuffleboardPage();
    }

    /** Update Shuffleboard Pages. This method will be called once per scheduler run
     * (=50Hz) */
    @Override
    public void periodic() {

        // update main page
        // update remaining time in match (rounded to nearest second)
        m_selectedPath = (Integer)m_autonomousPath.getSelected();
        m_timeLeft.setDouble(Math.round(Timer.getMatchTime()));
    }

    
    /** returns delay for autonomous routines */
    public double getAutoDelay()
    {
        return m_delayTime.getDouble(0.0);
    }

    // -------------------- Shuffboard Methods --------------------


    /** Create main shuffleboard page
     * Typically contains autonomous commands and other top-level robot controls*/
    private void initializeMainShuffleboardPage() {

        // Create Main Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Drive Setup");
        m_autonomousPath = new SendableChooser<Integer>();

        // add autonomous commands to page -
        m_autonomousPath.addOption("Anywhere One Note",0);
        m_autonomousPath.addOption("Two Note",1);

        tab.add("Preround Paths", m_autonomousPath).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);
        m_delayTime = tab.add("Auto Delay Time", 0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1).withSize(1, 1).withProperties(Map.of("min_value", 0, "max_value", 10)).getEntry();

        // Uses auto generated constants to put git info on dashboard
        // Only updated once at the beginning
        ShuffleboardLayout BuildInfoLayout = tab.getLayout("Build Info", BuiltInLayouts.kList);
        BuildInfoLayout.withPosition(6, 0);
        BuildInfoLayout.withSize(1, 3);
        BuildInfoLayout.add("Deployed Branch", BuildConstants.GIT_BRANCH);
        BuildInfoLayout.add("Build Timestamp", BuildConstants.BUILD_DATE);
        BuildInfoLayout.add("Repository", BuildConstants.MAVEN_NAME);    
        
        EffectorTarget = tab.add("Effector Setpoint", 0.05)
        .withPosition(3, 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min_value", CassetteEffector.MIN_BOTTOM_ANGLE, "max_value", CassetteEffector.MAX_TOP_ANGLE))
        .getEntry();

        ShotCharacterizationTargetL = tab.add("L Shooter Target", 1000)
        .withPosition(2, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

        ShotCharacterizationTargetR = tab.add("R Shooter Target", 1000)
        .withPosition(2, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();

        LShooterSpeed = tab.add("LShooterspeed", 0)
        .withPosition(3, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
        .getEntry();

        RShooterSpeed = tab.add("RShooterspeed", 0)
        .withPosition(3, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
        .getEntry();

        DistanceAdjustment = tab.addPersistent("Distance Adjustment", 0.05)
        .withPosition(3, 3)
        .withSize(1, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min_value", 0, "max_value", 20))
        .getEntry();

        ShuffleboardLayout ShotInfoLayout = tab.getLayout("Shot Info", BuiltInLayouts.kList)
        .withPosition(4, 0)
        .withSize(2, 4);
        ShooterAtSpeed = ShotInfoLayout.add("Shooter Speed", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

        ShooterAtAngle = ShotInfoLayout.add("Shooter Angle", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

        RobotAtAngle = ShotInfoLayout.add("Robot Angle", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

        SeesTarget = ShotInfoLayout.add("Target Aquired", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry();

        TargetDistance = ShotInfoLayout.add("Target Distance", 0).getEntry();

        // add match time remaining in autonomous/teleop part of match (seconds)
        ShuffleboardLayout l1 = tab.getLayout("Timer", BuiltInLayouts.kList);
        l1.withPosition(0, 2);
        l1.withSize(1, 2);
        m_timeLeft = l1.add("TimeLeft", 0.0).getEntry();
    }

    // returns position of autonomous commands on shuffleboard
    // typically called by Robot AutonomousInit to select auto path to be followed
    // returns true if selected, false if not
    // TODO <to be revised for 2022 robot>



} // end class ShuffleboardOI
