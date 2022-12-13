package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.Maths;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A command that locks the robot on the hub's position.
 */
public class LockOnHubCommand extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final PIDController pidController;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final DoubleSupplier targetSupplier;

    /**
     * Constructs a new LockOnHubCommand.
     *
     * @param pidController     the PIDController to use
     * @param robotPoseSupplier the robot's current position from the odometry
     * @param targetSupplier    the target degrees from the hub
     */
    public LockOnHubCommand(
            PIDController pidController, Supplier<Pose2d> robotPoseSupplier, DoubleSupplier targetSupplier) {
        this.pidController = pidController;
        this.robotPoseSupplier = robotPoseSupplier;
        this.targetSupplier = targetSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        pidController.setSetpoint(targetSupplier.getAsDouble());

        swerve.selfRelativeDrive(new Translation2d(), getAsRotation2dToTurn());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    private Rotation2d getAsRotation2dToTurn() {
        final double degreesFromHub = Maths.getRelativeAngleFromTranslation(
                robotPoseSupplier.get(), SwerveConstants.HUB_POSE.getTranslation());

        return new Rotation2d(pidController.calculate(degreesFromHub));
    }
}
