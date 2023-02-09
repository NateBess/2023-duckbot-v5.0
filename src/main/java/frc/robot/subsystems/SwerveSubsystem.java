package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    //         new Rotation2d(0), null);
    //FIXME Placed the SwerveModulePosition's that I created in this spot...
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), new SwerveModulePosition[] {
                frontLeft.getModulePosition(),
                frontRight.getModulePosition(),
                backLeft.getModulePosition(),
                backRight.getModulePosition()
              });

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        // odometer.resetPosition(pose, getRotation2d());
        // FIXME Placed the SwerveModulePosition's that I created in this spot...
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
          }, pose);
    }

    @Override
    public void periodic() {
        // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        //         backRight.getState());
        // FIXME Placed the SwerveModulePosition's that I created in this spot...
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
          });
        SmartDashboard.putNumber("Front Left Absolute Encoder", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Back Left Absolute Encoder", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Front Right Absolute Encoder", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Back Right Absolute Encoder", backRight.getTurningPosition());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // FIXME Figure out how to get the current ChassisSpeed for an input in this method

        // FIXME chassis speed goes in here... 
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, null, getHeading(), getHeading(), getHeading());
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}



//make a new method that takes in chassis speeds and at the end use normalize wheel speeds