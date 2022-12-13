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
 * A command that drives the robot while locking on the hub's position.
 */
public class DriveWithLockOnHubCommand extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final PIDController pidController;
    private final DoubleSupplier xPower, yPower;
    private final DoubleSupplier targetSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;

    /**
     * Constructs a new DriveWithLockOnHubCommand.
     *
     * @param pidController     the PIDController to use
     * @param xPower            the forwards power in meters per second
     * @param yPower            the leftwards power in meters per second
     * @param robotPoseSupplier the robot's current position from the odometry
     * @param targetSupplier    the target degrees from the hub
     */
    public DriveWithLockOnHubCommand(
            PIDController pidController, DoubleSupplier xPower, DoubleSupplier yPower,
            Supplier<Pose2d> robotPoseSupplier, DoubleSupplier targetSupplier) {
        this.pidController = pidController;
        this.xPower = xPower;
        this.yPower = yPower;
        this.targetSupplier = targetSupplier;
        this.robotPoseSupplier = robotPoseSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        pidController.setSetpoint(targetSupplier.getAsDouble());

        swerve.selfRelativeDrive(getPowerAsTranslation2d(), getAsRotation2dToTurn());
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

    private Translation2d getPowerAsTranslation2d() {
        return new Translation2d(
                xPower.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                yPower.getAsDouble() * SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
    }
}
