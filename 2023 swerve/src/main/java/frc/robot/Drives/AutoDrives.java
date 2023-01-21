package frc.robot.Drives;

import java.util.TimerTask;
import java.util.Timer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightReader;

public class AutoDrives extends CommandBase {

  
  private final Drivetrain mDrivetrain;
  private final double mTranslationXSupplier;
  private final double mTranslationYSupplier;
  private final double mRotationSupplier;
  


  public AutoDrives(Drivetrain drivetrainSubsystem, double desiredX,
      double desiredY, double desiredZ) {
    mDrivetrain = drivetrainSubsystem;
    mTranslationXSupplier = desiredX;
    mTranslationYSupplier = desiredY;
    mRotationSupplier = desiredZ;

    addRequirements(drivetrainSubsystem);
    
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.drive(GetNonRelativeSpeed(mTranslationXSupplier, mTranslationYSupplier, mRotationSupplier));

  }

  public ChassisSpeeds GetNonRelativeSpeed(double x, double y, double rot)
  {
    ChassisSpeeds Ret = new ChassisSpeeds(x,y,rot);
    return Ret;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
