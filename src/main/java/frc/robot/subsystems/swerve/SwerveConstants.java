package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.util.List;

public class SwerveConstants {
    static final double
            HUB_X = 8.2296,
            HUB_Y = 0.5121;
    static final Pose2d HUB_POSE = new Pose2d(HUB_X, HUB_Y, new Rotation2d());
    static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;
    static final double DRIVE_RAMP_RATE = 0.3;
    static final double DEAD_BAND_DRIVE_DEADBAND = 0.1;
    private static final Translation2d[] LOCATIONS = {
            SwerveModuleConstants.SwerveModules.fromId(0).location,
            SwerveModuleConstants.SwerveModules.fromId(1).location,
            SwerveModuleConstants.SwerveModules.fromId(2).location,
            SwerveModuleConstants.SwerveModules.fromId(3).location
    };
    private static final int PIGEON_ID = 25;
    static final Pigeon2 GYRO = new Pigeon2(PIGEON_ID);
    static SwerveModule[] SWERVE_MODULES = {
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(0).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(1).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(2).swerveModuleConstants),
            new SwerveModule(SwerveModuleConstants.SwerveModules.fromId(3).swerveModuleConstants)
    };
    static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);
    static final List<Pose3d> TAG_POSES = List.of(
            new Pose3d(),
            new Pose3d());
    static final double ALLOWED_TAG_AMBIGUITY = 0.2;

    /**
     * The vector represents how much the pose estimator can trust the process in each value.
     *
     * <p> The first value represents how much it can trust for the x,
     * the second one for the y, and the third one for the theta (rotation).
     */
    static final Vector<N3>
            MODULE_STATES_AMBIGUITY = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VISION_CALCULATIONS_AMBIGUITY = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    /**
     * The vector represents how much the pose estimator can trust the sensor readings from encoders and gyros.
     */
    static final Vector<N1> ENCODER_AND_GYRO_READINGS_AMBIGUITY = VecBuilder.fill(0.1);

    /**
     * The transformation between the camera to the robot.
     * <p>
     * The X and Y values for the Translation3d should be how far (in meters) the camera is from the robot.
     */
    static final Transform3d CAMERA_TO_ROBOT = new Transform3d(new Translation3d(), new Rotation3d());
}
