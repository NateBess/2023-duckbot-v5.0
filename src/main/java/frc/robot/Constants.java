// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1 / 6.67;
    public static final double kTurningMotorGearRatio = 1 / 1;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RADPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPTurning = 0.5;
  }

  public static class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(21.875);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(24.125);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final int kArmMotorPort = 9;

    public static final int kFrontLeftDriveMotorPort = 7;
    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kFrontRightDriveMotorPort = 1;
    public static final int kBackRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 8;
    public static final int kBackLeftTurningMotorPort = 6;
    public static final int kFrontRightTurningMotorPort = 2;
    public static final int kBackRightTurningMotorPort = 4;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
    public static final int kBackRightDriveAbsoluteEncoderPort = 1;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    //

    public static final double kFrontLeftAbsoluteEncoderRawValue = 1085;
    public static final double kBackLeftAbsoluteEncoderRawValue = 2970;
    public static final double kFrontRightAbsoluteEncoderRawValue = 3129;
    public static final double kBackRightAbsoluteEncoderRawValue = 3573;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetDegrees = ((kFrontLeftAbsoluteEncoderRawValue/4060)*360);  // 31.08;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetDegrees = ((kBackLeftAbsoluteEncoderRawValue/4060)*360);   // 66.75;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetDegrees = ((kFrontRightAbsoluteEncoderRawValue/4060)*360); // 13.75;
    public static final double kBackRightDriveAbsoluteEncoderOffsetDegrees = ((kBackRightAbsoluteEncoderRawValue/4060)*360);  //233.15;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
  }

  public static class OperatorConstants {
    //needs to be updated 
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.05; 
  }
}
