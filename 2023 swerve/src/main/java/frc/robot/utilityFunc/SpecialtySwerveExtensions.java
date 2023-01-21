// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilityFunc;

import static java.util.Objects.requireNonNull;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Utility class for common WPILib error messages. */
public final class SpecialtySwerveExtensions {
  /** Utility class, so constructor is private. */
  private SpecialtySwerveExtensions() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Requires that a parameter of a method not be null; prints an error message with helpful
   * debugging instructions if the parameter is null.
   *
   * @param <T> Type of object.
   * @param obj The parameter that must not be null.
   * @param paramName The name of the parameter.
   * @param methodName The name of the method.
   * @return The object parameter confirmed not to be null.
   */
  public static SwerveModulePosition getSwerveModulePosition(SwerveModule obj, int CANBUSID) {
    return new SwerveModulePosition(CreateTempCanCoderFromIDAndGrabDistance(CANBUSID), new Rotation2d(obj.getSteerAngle()));
  }

  public static double CreateTempCanCoderFromIDAndGrabDistance(int CANBUSID)
  {
    CANCoder temp = new CANCoder(CANBUSID);
    return temp.getAbsolutePosition();
  }
}