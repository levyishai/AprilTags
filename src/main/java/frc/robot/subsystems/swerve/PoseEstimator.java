package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimator extends SubsystemBase implements Loggable {
    private final Swerve swerve = Swerve.getInstance();
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private final PhotonCamera camera;
    @Log
    private final Field2d field = new Field2d();
    private double previousTimestamp = 0;

    public PoseEstimator(PhotonCamera camera) {
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerve.getHeading(),
                new Pose2d(),
                SwerveConstants.KINEMATICS,
                PoseEstimatorConstants.MODULE_STATES_AMBIGUITY,
                PoseEstimatorConstants.ENCODER_AND_GYRO_READINGS_AMBIGUITY,
                PoseEstimatorConstants.VISION_CALCULATIONS_AMBIGUITY
        );

        this.camera = camera;
    }

    public void resetOdometry(Pose2d startingPose) {
        swerveDrivePoseEstimator.resetPosition(startingPose, swerve.getHeading());
    }

    public Pose2d getCurrentPose() {
        if (swerveDrivePoseEstimator == null) return new Pose2d();

        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    void attemptToAddVisionMeasurement() {
        final PhotonPipelineResult pipelineResult = camera.getLatestResult();
        final double pipelineResultTimestamp = pipelineResult.getTimestampSeconds();

        if (pipelineResultTimestamp == previousTimestamp || !pipelineResult.hasTargets()) {
            updatePoseEstimator();
            return;
        }

        previousTimestamp = pipelineResultTimestamp;
        final PhotonTrackedTarget bestTag = pipelineResult.getBestTarget();
        final int bestTagId = bestTag.getFiducialId();

        if (bestTag.getPoseAmbiguity() > PoseEstimatorConstants.MINIMUM_TAG_AMBIGUITY || bestTagId < 0 ||
                bestTagId > PoseEstimatorConstants.TAG_POSES.size()) {
            updatePoseEstimator();
            return;
        }

        swerveDrivePoseEstimator.addVisionMeasurement(getRobotPoseFromTag(bestTag), pipelineResultTimestamp);
        updatePoseEstimator();
    }

    private Pose2d getRobotPoseFromTag(PhotonTrackedTarget tag) {
        final int tagId = tag.getFiducialId();

        final Transform3d
                cameraToTag = tag.getBestCameraToTarget(),
                tagToCamera = cameraToTag.inverse();

        final Pose3d
                tagPose = PoseEstimatorConstants.TAG_POSES.get(tagId),
                cameraPose = tagPose.transformBy(tagToCamera);

        return cameraPose.transformBy(PoseEstimatorConstants.CAMERA_TO_ROBOT).toPose2d();
    }

    private void updatePoseEstimator() {
        field.setRobotPose(getCurrentPose());
        swerveDrivePoseEstimator.update(swerve.getHeading(), swerve.getSwerveModuleStates());
    }

}
