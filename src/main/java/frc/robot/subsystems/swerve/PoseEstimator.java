package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimator extends SubsystemBase {
    private final Swerve swerve = Swerve.getInstance();
    private final PhotonCamera photonCamera;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private double previousTimestamp = 0;

    public PoseEstimator(PhotonCamera photonCamera) {
        this.photonCamera = photonCamera;
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerve.getHeading(),
                new Pose2d(),
                SwerveConstants.KINEMATICS,
                SwerveConstants.MODULE_STATES_TRUST,
                SwerveConstants.ENCODER_AND_GYRO_READINGS_TRUST,
                SwerveConstants.VISION_CALCULATIONS_TRUST);
    }

    @Override
    public void periodic() {
        final PhotonPipelineResult pipelineResult = photonCamera.getLatestResult();
        final double pipelineResultTimestamp = pipelineResult.getTimestampSeconds();

        if (pipelineResultTimestamp == previousTimestamp || !pipelineResult.hasTargets()) {
            updatePoseEstimator();
            return;
        }

        previousTimestamp = pipelineResultTimestamp;
        final PhotonTrackedTarget bestTarget = pipelineResult.getBestTarget();
        final int bestTargetTagId = bestTarget.getFiducialId();

        if (bestTarget.getPoseAmbiguity() <= 0.2 && bestTargetTagId >= 0 && bestTargetTagId < SwerveConstants.TAG_POSES.size()) {
            final Pose3d targetTagPose = SwerveConstants.TAG_POSES.get(bestTargetTagId);
            final Transform3d camToTarget = bestTarget.getBestCameraToTarget();
            final Pose3d
                    camPose = targetTagPose.transformBy(camToTarget.inverse()),
                    visionMeasurement = camPose.transformBy(SwerveConstants.CAMERA_TO_ROBOT);

            swerveDrivePoseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), pipelineResultTimestamp);
        }

        updatePoseEstimator();
    }

    public void resetOdometry(Pose2d startingPose) {
        swerveDrivePoseEstimator.resetPosition(startingPose, swerve.getHeading());
    }

    public Pose2d getCurrentPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    private void updatePoseEstimator() {
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getSwerveModuleStates());
    }
}
