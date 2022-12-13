package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

public class SwerveModuleConstants {
    final WPI_TalonFX driveMotor, angleMotor;
    final WPI_TalonSRX angleEncoder;
    final double encoderOffset;

    SwerveModuleConstants(
            WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, WPI_TalonSRX angleEncoder, double encoderOffset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.angleEncoder = angleEncoder;
        this.encoderOffset = encoderOffset;
    }

    static final double DRIVE_GEAR_RATIO = 8.14;
    private static final double VOLTAGE_COMP_SATURATION = 12;
    private static final double
            FRONT_LEFT_ENCODER_OFFSET = 3501,
            FRONT_RIGHT_ENCODER_OFFSET = 2627,
            RIGHT_LEFT_ENCODER_OFFSET = 1673,
            REAR_RIGHT_ENCODER_OFFSET = 315;
    private static final double
            WHEEL_DIAMETER_METERS = 0.1,
            SIDE_LENGTH_METERS = 0.5;
    static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    private static final double DISTANCE_FROM_CENTER_OF_BASE = SIDE_LENGTH_METERS / 2;
    private static final Translation2d
            FRONT_LEFT_MODULE_LOCATION = new Translation2d(
                    DISTANCE_FROM_CENTER_OF_BASE,
                    DISTANCE_FROM_CENTER_OF_BASE
            ),
            FRONT_RIGHT_MODULE_LOCATION = new Translation2d(
                    DISTANCE_FROM_CENTER_OF_BASE,
                    -DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_LEFT_MODULE_LOCATION = new Translation2d(
                    -DISTANCE_FROM_CENTER_OF_BASE,
                    DISTANCE_FROM_CENTER_OF_BASE
            ),
            REAR_RIGHT_MODULE_LOCATION = new Translation2d(
                    -DISTANCE_FROM_CENTER_OF_BASE,
                    DISTANCE_FROM_CENTER_OF_BASE
            );
    private static final int
            FRONT_LEFT_DRIVE_MOTOR_ID = 17,
            FRONT_RIGHT_DRIVE_MOTOR_ID = 1,
            REAR_LEFT_DRIVE_MOTOR_ID = 15,
            REAR_RIGHT_DRIVE_MOTOR_ID = 2,
            FRONT_LEFT_ANGLE_MOTOR_ID = 19,
            FRONT_RIGHT_ANGLE_MOTOR_ID = 0,
            REAR_LEFT_ANGLE_MOTOR_ID = 14,
            REAR_RIGHT_ANGLE_MOTOR_ID = 4,
            FRONT_LEFT_ANGLE_ENCODER_ID = 13,
            FRONT_RIGHT_ANGLE_ENCODER_ID = 5,
            REAR_LEFT_ANGLE_ENCODER_ID = 16,
            REAR_RIGHT_ANGLE_ENCODER_ID = 3,
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            REAR_LEFT_ID = 2,
            REAR_RIGHT_ID = 3;
    private static final boolean
            FRONT_LEFT_DRIVE_MOTOR_INVERTED = true,
            FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true,
            REAR_LEFT_DRIVE_MOTOR_INVERTED = true,
            REAR_RIGHT_DRIVE_MOTOR_INVERTED = true,
            FRONT_LEFT_ANGLE_MOTOR_INVERTED = false,
            FRONT_RIGHT_ANGLE_MOTOR_INVERTED = false,
            REAR_LEFT_ANGLE_MOTOR_INVERTED = false,
            REAR_RIGHT_ANGLE_MOTOR_INVERTED = false,
            FRONT_LEFT_ANGLE_ENCODER_INVERTED = false,
            FRONT_RIGHT_ANGLE_ENCODER_INVERTED = false,
            REAR_LEFT_ANGLE_ENCODER_INVERTED = false,
            REAR_RIGHT_ANGLE_ENCODER_INVERTED = false;
    private static final WPI_TalonSRX
            FRONT_LEFT_ANGLE_ENCODER = new WPI_TalonSRX(FRONT_LEFT_ANGLE_ENCODER_ID),
            FRONT_RIGHT_ANGLE_ENCODER = new WPI_TalonSRX(FRONT_RIGHT_ANGLE_ENCODER_ID),
            REAR_LEFT_ANGLE_ENCODER = new WPI_TalonSRX(REAR_LEFT_ANGLE_ENCODER_ID),
            REAR_RIGHT_ANGLE_ENCODER = new WPI_TalonSRX(REAR_RIGHT_ANGLE_ENCODER_ID);
    private static final List<WPI_TalonSRX> ANGLE_ENCODERS = List.of(
            FRONT_LEFT_ANGLE_ENCODER, FRONT_RIGHT_ANGLE_ENCODER, REAR_LEFT_ANGLE_ENCODER, REAR_RIGHT_ANGLE_ENCODER);
    private static final WPI_TalonFX
            FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(FRONT_LEFT_DRIVE_MOTOR_ID),
            FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(FRONT_RIGHT_DRIVE_MOTOR_ID),
            REAR_LEFT_DRIVE_MOTOR = new WPI_TalonFX(REAR_LEFT_DRIVE_MOTOR_ID),
            REAR_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(REAR_RIGHT_DRIVE_MOTOR_ID),
            FRONT_LEFT_ANGLE_MOTOR = new WPI_TalonFX(FRONT_LEFT_ANGLE_MOTOR_ID),
            FRONT_RIGHT_ANGLE_MOTOR = new WPI_TalonFX(FRONT_RIGHT_ANGLE_MOTOR_ID),
            REAR_LEFT_ANGLE_MOTOR = new WPI_TalonFX(REAR_LEFT_ANGLE_MOTOR_ID),
            REAR_RIGHT_ANGLE_MOTOR = new WPI_TalonFX(REAR_RIGHT_ANGLE_MOTOR_ID);
    private static final List<WPI_TalonFX>
            DRIVE_MOTORS = List.of(FRONT_LEFT_DRIVE_MOTOR, FRONT_RIGHT_DRIVE_MOTOR,
                    REAR_LEFT_DRIVE_MOTOR, REAR_RIGHT_DRIVE_MOTOR),
            ANGLE_MOTORS = List.of(FRONT_LEFT_ANGLE_MOTOR, FRONT_RIGHT_ANGLE_MOTOR,
                    REAR_LEFT_ANGLE_MOTOR, REAR_RIGHT_ANGLE_MOTOR);
    private static final SwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    FRONT_LEFT_DRIVE_MOTOR,
                    FRONT_LEFT_ANGLE_MOTOR,
                    FRONT_LEFT_ANGLE_ENCODER,
                    FRONT_LEFT_ENCODER_OFFSET
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_ANGLE_MOTOR,
                    FRONT_RIGHT_ANGLE_ENCODER,
                    FRONT_RIGHT_ENCODER_OFFSET
            ),
            REAR_LEFT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    REAR_LEFT_DRIVE_MOTOR,
                    REAR_LEFT_ANGLE_MOTOR,
                    REAR_LEFT_ANGLE_ENCODER,
                    RIGHT_LEFT_ENCODER_OFFSET
            ),
            REAR_RIGHT_SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants(
                    REAR_RIGHT_DRIVE_MOTOR,
                    REAR_RIGHT_ANGLE_MOTOR,
                    REAR_RIGHT_ANGLE_ENCODER,
                    REAR_RIGHT_ENCODER_OFFSET
            );
    private static final PID
            FRONT_LEFT_PID = new PID(0.7, 0, 0),
            FRONT_RIGHT_PID = new PID(0.7, 0, 0),
            REAR_LEFT_PID = new PID(0.7, 0, 0),
            REAR_RIGHT_PID = new PID(0.7, 0, 0);

    static {
        for (WPI_TalonFX currentDriveMotor : DRIVE_MOTORS) {
            currentDriveMotor.configFactoryDefault();
            currentDriveMotor.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
            currentDriveMotor.enableVoltageCompensation(true);
            currentDriveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
            currentDriveMotor.configOpenloopRamp(SwerveConstants.DRIVE_RAMP_RATE);
        }

        for(WPI_TalonFX currentAngleMotor : ANGLE_MOTORS) {
            currentAngleMotor.configFactoryDefault();
            currentAngleMotor.configVoltageCompSaturation(VOLTAGE_COMP_SATURATION);
            currentAngleMotor.enableVoltageCompensation(true);
            currentAngleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 1000);
            currentAngleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
        }

        for (WPI_TalonSRX currentAngleEncoder : ANGLE_ENCODERS) {
            currentAngleEncoder.configFactoryDefault();
            currentAngleEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute.toFeedbackDevice());
            currentAngleEncoder.configFeedbackNotContinuous(false, 0);
        }

        FRONT_LEFT_DRIVE_MOTOR.setInverted(FRONT_LEFT_DRIVE_MOTOR_INVERTED);
        FRONT_RIGHT_DRIVE_MOTOR.setInverted(FRONT_RIGHT_DRIVE_MOTOR_INVERTED);
        REAR_LEFT_DRIVE_MOTOR.setInverted(REAR_LEFT_DRIVE_MOTOR_INVERTED);
        REAR_RIGHT_DRIVE_MOTOR.setInverted(REAR_RIGHT_DRIVE_MOTOR_INVERTED);

        FRONT_LEFT_ANGLE_MOTOR.setInverted(FRONT_LEFT_ANGLE_MOTOR_INVERTED);
        FRONT_RIGHT_ANGLE_MOTOR.setInverted(FRONT_RIGHT_ANGLE_MOTOR_INVERTED);
        REAR_LEFT_ANGLE_MOTOR.setInverted(REAR_LEFT_ANGLE_MOTOR_INVERTED);
        REAR_RIGHT_ANGLE_MOTOR.setInverted(REAR_RIGHT_ANGLE_MOTOR_INVERTED);

        FRONT_LEFT_ANGLE_ENCODER.setInverted(FRONT_LEFT_ANGLE_ENCODER_INVERTED);
        FRONT_RIGHT_ANGLE_MOTOR.setInverted(FRONT_RIGHT_ANGLE_ENCODER_INVERTED);
        REAR_LEFT_ANGLE_ENCODER.setInverted(REAR_LEFT_ANGLE_ENCODER_INVERTED);
        REAR_RIGHT_ANGLE_ENCODER.setInverted(REAR_RIGHT_ANGLE_ENCODER_INVERTED);

        FRONT_LEFT_ANGLE_MOTOR.config_kP(0, FRONT_LEFT_PID.p);
        FRONT_LEFT_ANGLE_MOTOR.config_kI(0, FRONT_LEFT_PID.i);
        FRONT_LEFT_ANGLE_MOTOR.config_kD(0, FRONT_LEFT_PID.d);

        FRONT_RIGHT_ANGLE_MOTOR.config_kP(0, FRONT_RIGHT_PID.p);
        FRONT_RIGHT_ANGLE_MOTOR.config_kI(0, FRONT_RIGHT_PID.i);
        FRONT_RIGHT_ANGLE_MOTOR.config_kD(0, FRONT_RIGHT_PID.d);

        REAR_LEFT_ANGLE_MOTOR.config_kP(0, REAR_LEFT_PID.p);
        REAR_LEFT_ANGLE_MOTOR.config_kI(0, REAR_LEFT_PID.i);
        REAR_LEFT_ANGLE_MOTOR.config_kD(0, REAR_LEFT_PID.d);

        REAR_RIGHT_ANGLE_MOTOR.config_kP(0, REAR_RIGHT_PID.p);
        REAR_RIGHT_ANGLE_MOTOR.config_kI(0, REAR_RIGHT_PID.i);
        REAR_RIGHT_ANGLE_MOTOR.config_kD(0, REAR_RIGHT_PID.d);
    }

    /**
     * An enum for the swerve modules.
     */
    enum SwerveModules {
        FRONT_LEFT(FRONT_LEFT_ID, FRONT_LEFT_SWERVE_MODULE_CONSTANTS, FRONT_LEFT_MODULE_LOCATION),
        FRONT_RIGHT(FRONT_RIGHT_ID, FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, FRONT_RIGHT_MODULE_LOCATION),
        REAR_LEFT(REAR_LEFT_ID, REAR_LEFT_SWERVE_MODULE_CONSTANTS, REAR_LEFT_MODULE_LOCATION),
        REAR_RIGHT(REAR_RIGHT_ID, REAR_RIGHT_SWERVE_MODULE_CONSTANTS, REAR_RIGHT_MODULE_LOCATION);

        final int id;
        final SwerveModuleConstants swerveModuleConstants;
        final Translation2d location;

        SwerveModules(int id, SwerveModuleConstants swerveModuleConstants, Translation2d location) {
            this.id = id;
            this.swerveModuleConstants = swerveModuleConstants;
            this.location = location;
        }

        static SwerveModules fromId(int id) {
            return values()[id];
        }
    }

    private static class PID {
        private final double p, i, d;

        private PID(double p, double i, double d){
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }
}
