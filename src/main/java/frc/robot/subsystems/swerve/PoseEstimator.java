package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                SwerveConstants.MODULE_STATES_AMBIGUITY,
                SwerveConstants.ENCODER_AND_GYRO_READINGS_AMBIGUITY,
                SwerveConstants.VISION_CALCULATIONS_AMBIGUITY);

        putOnDashboard();
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
        final PhotonTrackedTarget bestTag = pipelineResult.getBestTarget();
        final int bestTagId = bestTag.getFiducialId();

        if (bestTag.getPoseAmbiguity() > SwerveConstants.ALLOWED_TAG_AMBIGUITY || bestTagId < 0 ||
                bestTagId > SwerveConstants.TAG_POSES.size()){
            updatePoseEstimator();
            return;
        }

        final Pose3d bestTagPose = SwerveConstants.TAG_POSES.get(bestTagId);
        final Transform3d
                cameraToTag = bestTag.getBestCameraToTarget(),
                tagToCamera = cameraToTag.inverse();
        final Pose3d
                cameraPose = bestTagPose.transformBy(tagToCamera),
                robotPose = cameraPose.transformBy(SwerveConstants.CAMERA_TO_ROBOT);

        swerveDrivePoseEstimator.addVisionMeasurement(robotPose.toPose2d(), pipelineResultTimestamp);
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

        putOnDashboard();
    }

    private void putOnDashboard() {
        SmartDashboard.putNumber("robot_x", getCurrentPose().getX());
        SmartDashboard.putNumber("robot_y", getCurrentPose().getY());
        SmartDashboard.putNumber("robot_degrees", getCurrentPose().getRotation().getDegrees());
    }
}
