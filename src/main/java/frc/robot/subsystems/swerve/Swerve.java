package frc.robot.subsystems.swerve;


import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class Swerve extends SubsystemBase {
    private final static Swerve INSTANCE = new Swerve();
    private final Pigeon2 gyro = SwerveConstants.GYRO;

    private Swerve() {
        zeroHeading();
    }

    public static Swerve getInstance() {
        return INSTANCE;
    }

    /**
     * Stops all swerve modules.
     */
    void stop() {
        for (SwerveModule module : SwerveConstants.SWERVE_MODULES) {
            module.stopModule();
        }
    }

    /**
     * Drives the robot's swerve modules with the given velocities, relative to itself.
     *
     * @param translation2d the target x and y velocities in meters per second
     * @param rotation2d    the target theta velocity in radians per second
     */
    void selfRelativeDrive(Translation2d translation2d, Rotation2d rotation2d) {
        final ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                translation2d.getX(),
                translation2d.getY(),
                rotation2d.getRadians()
        );

        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * Drives the robot's swerve modules with the given velocities, relative to the field.
     *
     * @param translation2d the target x and y velocities in meters per second
     * @param rotation2d    the target theta velocity in radians per second
     */
    void fieldRelativeDrive(Translation2d translation2d, Rotation2d rotation2d) {
        final ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation2d.getX(),
                translation2d.getY(),
                rotation2d.getRadians(),
                getHeading()
        );

        selfRelativeDrive(chassisSpeeds);
    }

    /**
     * @return the gyro's yaw in Rotation2d
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        final List<SwerveModuleState> swerveModuleStates = new ArrayList<>();

        for(SwerveModule currentModule : SwerveConstants.SWERVE_MODULES) {
            swerveModuleStates.add(currentModule.getCurrentState());
        }

        return swerveModuleStates.toArray(SwerveModuleState[]::new);
    }

    private void zeroHeading() {
        setHeading(0);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] targetSwerveModuleState = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setTargetModuleStates(targetSwerveModuleState);
    }

    private void setTargetModuleStates(SwerveModuleState[] targetSwerveModuleStates) {
        for (int i = 0; i < 4; i++) {
            SwerveConstants.SWERVE_MODULES[i].setTargetState(targetSwerveModuleStates[i]);
        }
    }

    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) < SwerveConstants.DEAD_BAND_DRIVE_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) < SwerveConstants.DEAD_BAND_DRIVE_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) < SwerveConstants.DEAD_BAND_DRIVE_DEADBAND;
    }

    private void setHeading(double yaw) {
        gyro.setYaw(yaw);
    }
}

