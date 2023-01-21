package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightReader;
import frc.robot.utilityFunc.CrashTracker;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.AutoModeExecutor;
import frc.robot.Auto.AutoModeSelector;
import frc.robot.Auto.Modes.AutoModeBase;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer mRobotContainer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

    String trajectoryJSON = "paths/Testing.wpilib.json";
Trajectory trajectory = new Trajectory();
private Command m_autonomousCommand;
	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

@Override
public void robotInit() {
   try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

    mRobotContainer = new RobotContainer();
    LimelightReader.InitLimeLight();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    LimelightReader.Instance.UpdateLimeCam();
    LimelightReader.Instance.UpdateSmartDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    CrashTracker.logAutoInit();

		try {

			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				Drivetrain.getInstance().resetOdometry(autoMode.get().getStartingPose());
			}

			mAutoModeExecutor.start();

			//.LimelightReader.Instance().setPipeline(Constants.VisionConstants.kDefaultPipeline);

			// set champs pride automation
			//mLEDs.setChampsAutoAnimation();	

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    Drivetrain.CurrentBotState = Drivetrain.DefaultTeleopState;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
