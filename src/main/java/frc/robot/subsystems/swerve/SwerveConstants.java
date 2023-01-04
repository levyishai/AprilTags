package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import io.github.oblarg.oblog.Loggable;

public class SwerveConstants implements Loggable {
    static final double
            HUB_X = 8.2296,
            HUB_Y = 0.5121;
    static final Pose2d HUB_POSE = new Pose2d(HUB_X, HUB_Y, new Rotation2d());
    static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    static final double DRIVE_RAMP_RATE = 0.3;
    static final double DEAD_BAND_DRIVE_DEADBAND = 0.1;
    static final SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(0).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(1).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(2).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(3).swerveModuleConstants)
    };
    private static final Translation2d[] LOCATIONS = {
            SwerveModuleConstants.SwerveModules.fromId(0).location,
            SwerveModuleConstants.SwerveModules.fromId(1).location,
            SwerveModuleConstants.SwerveModules.fromId(2).location,
            SwerveModuleConstants.SwerveModules.fromId(3).location
    };
    static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    private static final int PIGEON_ID = 25;
    static final Pigeon2 GYRO = new Pigeon2(PIGEON_ID);
}