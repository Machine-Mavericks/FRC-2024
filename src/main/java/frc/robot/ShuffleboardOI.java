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


/** Contains shuffleboard setup for generic main page not belonging to any subsubsystem
 * Typically used to have robot-level settings and command functions */
public class ShuffleboardOI extends SubsystemBase {

    // example autonomous path shuffleboard selection boxes
    // true if selected, false if not
    // <add any other controls here that go on main shufflebard page
    private GenericEntry m_delayTime;
    public SendableChooser<Integer> m_autonomousPath;

    // Shot info
    public GenericEntry LShooterSpeed;
    public GenericEntry RShooterSpeed;
    public GenericEntry LShooterTarget;
    public GenericEntry RShooterTarget;
    public GenericEntry LShooterError;
    public GenericEntry RShooterError;
    public GenericEntry ShooterAtSpeed;
    public GenericEntry ShooterAtAngle;

    public GenericEntry DistanceAdjustment;

    // other controls on main page
    private GenericEntry m_timeLeft;
    //private Integer m_selectedPath;

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
        //m_selectedPath = (Integer)m_autonomousPath.getSelected();
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
        m_autonomousPath.addOption("One Note Anywhere",0);
        m_autonomousPath.addOption("Two Note Amp",1);
        m_autonomousPath.addOption("Two Note Center", 2);
        m_autonomousPath.addOption("Five Note Amp",3);
        m_autonomousPath.addOption("Four Note Source",4);
        m_autonomousPath.addOption("Do Nothing",5);
        m_autonomousPath.addOption("Six Note Amp",6);
        m_autonomousPath.addOption("Two Note Source",7);
        m_autonomousPath.addOption("Three Note Source", 8);
        m_autonomousPath.addOption("Three Note Stage Center", 9);
        m_autonomousPath.addOption("Three Note Amp", null);
        m_autonomousPath.setDefaultOption("Do Nothing", 5);

        tab.add("Preround Paths", m_autonomousPath).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2,1);
        m_delayTime = tab.add("Auto Delay Time", 0).withWidget(BuiltInWidgets.kNumberSlider).withPosition(0, 1).withSize(1, 1).withProperties(Map.of("min_value", 0, "max_value", 15)).getEntry();

        // Uses auto generated constants to put git info on dashboard
        // Only updated once at the beginning
        ShuffleboardLayout BuildInfoLayout = tab.getLayout("Build Info", BuiltInLayouts.kList);
        BuildInfoLayout.withPosition(6, 0);
        BuildInfoLayout.withSize(1, 3);
        BuildInfoLayout.add("Deployed Branch", BuildConstants.GIT_BRANCH);
        BuildInfoLayout.add("Build Timestamp", BuildConstants.BUILD_DATE);
        BuildInfoLayout.add("Repository", BuildConstants.MAVEN_NAME);    
        
        LShooterSpeed = tab.add("L Shooter speed", 0)
        .withPosition(3, 1)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
        .getEntry();

        RShooterSpeed = tab.add("R Shooter speed", 0)
        .withPosition(3, 2)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
        .getEntry();

        RShooterTarget = tab.add("R Shooter target", 0)
        .withPosition(3, 3)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
        .getEntry();

        LShooterTarget = tab.add("L Shooter target", 0)
        .withPosition(3, 4)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
        .getEntry();

        RShooterError = tab.add("R Shooter error", 0)
        .withPosition(3, 5)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
        .getEntry();

        LShooterError = tab.add("L Shooter error", 0)
        .withPosition(3, 6)
        .withWidget(BuiltInWidgets.kTextView)
        .withProperties(Map.of("show_submit_button ", true))
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

        // add match time remaining in autonomous/teleop part of match (seconds)
        ShuffleboardLayout l1 = tab.getLayout("Timer", BuiltInLayouts.kList);
        l1.withPosition(0, 2);
        l1.withSize(1, 2);
        m_timeLeft = l1.add("TimeLeft", 0.0).getEntry();
    }


} // end class ShuffleboardOI
