package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.PoseEstimator;
import frc.robot.subsystems.swerve.PoseEstimatorUpdater;
import org.photonvision.PhotonCamera;


public class RobotContainer {
    private static final PhotonCamera CAMERA = new PhotonCamera("gloworm");
    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(CAMERA);
    private final Command poseEstimatorUpdater = new PoseEstimatorUpdater(POSE_ESTIMATOR);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        POSE_ESTIMATOR.setDefaultCommand(poseEstimatorUpdater);
    }

}
