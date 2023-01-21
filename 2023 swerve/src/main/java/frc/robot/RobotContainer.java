package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoDrives;
import frc.robot.commands.DrivetrainAuto;
import frc.robot.commands.DrivetrainTeleOp;

import frc.robot.subsystems.Drivetrain;


public class RobotContainer {
	private final CommandXboxController mXbox = new CommandXboxController(0);
	private final CommandJoystick mJoystick = new CommandJoystick(1);


	private final Drivetrain mDrivetrain = new Drivetrain();


	public RobotContainer() {

		mDrivetrain.setDefaultCommand(new DrivetrainTeleOp(
				mDrivetrain,
				() -> -modifyAxis(mXbox.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(mXbox.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(mXbox.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));


		configureBindings();
	}

	private void configureBindings() {
		mXbox.back().onTrue(mDrivetrain.zeroGyroscopeCommand());

	}
	

	public Command getAutonomousCommand() {
		final Command auto = new AutoDrives(mDrivetrain, 0, 0 ,0.5 );
		return auto;
  }


	private static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			if (value > 0.0) {
				return (value - deadband) / (1.0 - deadband);
			} else {
				return (value + deadband) / (1.0 - deadband);
			}
		} else {
			return 0.0;
		}
	}

	private static double modifyAxis(double value) {
		// Deadband
		value = deadband(value, 0.1);
		// Square the axis
		value = Math.copySign(value * value, value);

		return value;
	}

	public void zeroGyro() {
		mDrivetrain.zeroGyroscope();
	}


}