package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveJoystick;
import frc.robot.Constants.*;

public class swerveSubsystem extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegrees,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    public final SwerveModulePosition[] swervePos = new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), swervePos);

    public void SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {

            }
        }).start();
    }

    private void zeroHeading() {
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

    public void resetOdometer(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swervePos, pose);
    }

    @Override 
    public void periodic() {
        odometer.update(getRotation2d(), swervePos);
        SmartDashboard.putNumber("BackLeft Turning Angle", (backLeft.getTurningPosition()));
        SmartDashboard.putNumber("BackRight Turning Angle", (backRight.getTurningPosition()));
        SmartDashboard.putNumber("FrontLeft Turning Angle", (frontLeft.getTurningPosition()));
        SmartDashboard.putNumber("FrontRight Turning Angle", (frontRight.getTurningPosition()));

        SmartDashboard.putNumber("BackLeft Offset Degree", DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetDegrees);
        SmartDashboard.putNumber("BackRight Offset Degree", DriveConstants.kBackRightDriveAbsoluteEncoderOffsetDegrees);
        SmartDashboard.putNumber("FrontLeft Offset Degree", DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetDegrees);
        SmartDashboard.putNumber("FrontRight Offset Degree", DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetDegrees);
    }

    public void stopModule() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] dersiredStates) {
        frontLeft.setDesiredState(dersiredStates[0]);
        frontRight.setDesiredState(dersiredStates[1]);
        backLeft.setDesiredState(dersiredStates[2]);
        backRight.setDesiredState(dersiredStates[3]);
    }
}
