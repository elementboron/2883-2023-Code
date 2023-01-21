package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int LEFT_FRONT_DRIVE = 10, // DRIVETRAIN MOTORS
                          LEFT_REAR_DRIVE = 12,
                          RIGHT_FRONT_DRIVE = 16,
                          RIGHT_REAR_DRIVE = 14,
                          LEFT_FRONT_STEER = 11,
                          LEFT_REAR_STEER = 13,
                          RIGHT_FRONT_STEER = 17,
                          RIGHT_REAR_STEER = 15;

  public static final int LEFT_FRONT_ENCODER = 20, // CANCODERS
                          LEFT_REAR_ENCODER = 21,
                          RIGHT_FRONT_ENCODER = 23,
                          RIGHT_REAR_ENCODER = 22;

  public static final int CANDLE = 1;

  public static final int MAX_COUNTS_PER_REV = 42;
  public static final double EPSILON = 0.0001;
  
  // The left-to-right distance between the drivetrain wheels. Should be measured from center to center.
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6223; // Measure and set trackwidth
  // The front-to-back distance between the drivetrain wheels. Should be measured from center to center.
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.6223; // Measure and set wheelbase

  // FIXME: Calibrate these values often. Set all values to 0 to do so.
  public static final double LEFT_FRONT_STEER_OFFSET = -Math.toRadians(190); // Measure and set front left steer offset
  public static final double LEFT_REAR_STEER_OFFSET = -Math.toRadians(224); // Measure and set front right steer offset
  public static final double RIGHT_FRONT_STEER_OFFSET = -Math.toRadians(88.8); // Measure and set back left steer offset
  public static final double RIGHT_REAR_STEER_OFFSET = -Math.toRadians(34); // Measure and set back right steer offset

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  
}
