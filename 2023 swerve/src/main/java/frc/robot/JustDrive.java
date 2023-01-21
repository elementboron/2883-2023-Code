package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DrivetrainTeleOp;
import frc.robot.subsystems.Drivetrain;


public class JustDrive extends CommandBase {

    private final double ax;
    private final double ay;
    private final double az;
    private final Drivetrain mDrivetrain = new Drivetrain();
    private final CommandXboxController mXbox = new CommandXboxController(0);


    public JustDrive(double x, double y, double z){
        ax = x;
        ay = y;
        az = z;
        mDrivetrain.setDefaultCommand(new DrivetrainTeleOp(
				mDrivetrain,
				() -> -modifyAxis(ax) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(ay) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
				() -> -modifyAxis(az) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));


		configureBindings();

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

    private void configureBindings() {
    mXbox.back().onTrue(mDrivetrain.zeroGyroscopeCommand());

}

    


}
