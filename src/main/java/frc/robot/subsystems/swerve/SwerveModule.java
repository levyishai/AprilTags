package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utilities.Conversions;

public class SwerveModule {
    private final WPI_TalonFX driveMotor, angleMotor;
    private final WPI_TalonSRX angleEncoder;
    private final double encoderOffset;

    SwerveModule(SwerveModuleConstants constants) {
        this.driveMotor = constants.driveMotor;
        this.angleMotor = constants.angleMotor;
        this.angleEncoder = constants.angleEncoder;
        this.encoderOffset = constants.encoderOffset;

        configureRemoteSensor();
    }

    /**
     * Sets the target state of the swerve module.
     *
     * @param targetState the target state of the swerve module
     */
    void setTargetState(SwerveModuleState targetState) {
        final SwerveModuleState optimizedTargetState = optimizeState(targetState);
        final double targetAngle = optimizedTargetState.angle.getDegrees();
        final double velocity = optimizedTargetState.speedMetersPerSecond;

        setTargetAngleAndVelocity(targetAngle, velocity);
    }

    /**
     * Stops the swerve module.
     */
    void stopModule() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    /**
     * Gets the current state of the swerve module.
     *
     * @return the current state of the swerve module
     */
    SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentVelocity(), getCurrentAngle());
    }

    private SwerveModuleState optimizeState(SwerveModuleState state) {
        final double scopedDegrees = scope(state.angle.getDegrees());
        final double flippedDegrees = scope(180 + state.angle.getDegrees());
        final double scopedDiff = Math.abs(scopedDegrees - getCurrentDegrees());
        final double flippedDiff = Math.abs(flippedDegrees - getCurrentDegrees());

        if(scopedDiff < flippedDiff)
            return new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(scopedDegrees));

        return new SwerveModuleState(-state.speedMetersPerSecond, Rotation2d.fromDegrees(flippedDegrees));
    }

    private void setTargetAngleAndVelocity(double targetAngle, double velocity) {
        setTargetAngle(targetAngle);
        setTargetVelocity(velocity);
    }

    private void setTargetAngle(double targetAngle) {
        final double targetAnglePosition = Conversions.degreesToMagTicks(targetAngle);

        angleMotor.set(ControlMode.Position, targetAnglePosition + encoderOffset);
    }

    private void setTargetVelocity(double velocity) {
        final double targetPower = Conversions.velocityToTargetPower(velocity, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

        driveMotor.set(ControlMode.PercentOutput, targetPower);
    }

    private double getCurrentDegrees() {
        final double currentTicks = angleMotor.getSelectedSensorPosition() - encoderOffset;

        return Conversions.ticksToDegrees(currentTicks);
    }

    private double getCurrentVelocity() {
        final double ticksPer100Ms = driveMotor.getSelectedSensorVelocity();
        final double rotationPerSecond = Conversions.ticksPer100MsToRotationPerSecond(ticksPer100Ms);

        return Conversions.revolutionsToMeters(
                rotationPerSecond,
                SwerveModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                SwerveModuleConstants.DRIVE_GEAR_RATIO
        );
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(getCurrentDegrees());
    }

    private double scope(double targetAngle) {
        final double rawCurrentAngle = getCurrentDegrees() % 360;
        final double rawTargetAngle = targetAngle % 360;
        double difference = rawTargetAngle - rawCurrentAngle;

        if (difference < -180) {
            difference += 360;
        } else if (difference > 180) {
            difference -= 360;
        }

        return difference + getCurrentDegrees();
    }

    private void configureRemoteSensor() {
        angleMotor.configRemoteFeedbackFilter(angleEncoder, 0);
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    }
}
