package frc.robot.subsystems.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.util.List;

public class PoseEstimatorConstants {
    static final List<Pose3d> TAG_POSES = List.of(
            new Pose3d(),
            new Pose3d());
    static final double MINIMUM_TAG_AMBIGUITY = 0.2;

    /**
     * The vector represents how much the pose estimator can trust the process in each value.
     * <p>
     * The first value represents how much it can trust for the x,
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
