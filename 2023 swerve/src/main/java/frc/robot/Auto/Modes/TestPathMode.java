package frc.robot.Auto.Modes;

import frc.robot.Auto.Extensions.AutoModeEndedException;
import frc.robot.Auto.Extensions.AutoTrajectoryProvider;
import frc.robot.Constants;
import frc.robot.Auto.Actions.LambdaAction;
import frc.robot.Auto.Actions.SwerveTrajectoryAction;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class TestPathMode extends AutoModeBase {
    
    // Swerve instance 
    private final Drivetrain mSwerve = Drivetrain.getInstance();

    // required PathWeaver trajectory paths
    String path = "PathWeaver/Paths/Testing";
    
	// trajectories
	SwerveTrajectoryAction testTrajectoryAction;

    public TestPathMode() {

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    
        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path = AutoTrajectoryProvider.generateTrajectoryFromFile(path, Drivetrain.defaultSpeedConfig);
        testTrajectoryAction = new SwerveTrajectoryAction(traj_path,
                                                            mSwerve::getPose, Drivetrain.m_kinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(0.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);
		
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running test mode auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(testTrajectoryAction.getInitialPose())));

        runAction(testTrajectoryAction);
        
        System.out.println("Finished auto!");
    }

    @Override
    public Pose2d getStartingPose() {
        return testTrajectoryAction.getInitialPose();
    }
}


