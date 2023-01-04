package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PoseEstimatorUpdater extends CommandBase {
    private final PoseEstimator poseEstimator;

    public PoseEstimatorUpdater(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;

        addRequirements(poseEstimator);
    }

    @Override
    public void execute() {
        poseEstimator.attemptToAddVisionMeasurement();
    }
}
