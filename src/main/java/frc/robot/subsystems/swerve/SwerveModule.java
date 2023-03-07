package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.CANCODER_OFFSETS;
import static frc.robot.Constants.Swerve.DEGREES_TO_TICKS;
import static frc.robot.Constants.Swerve.METERS_TO_TICKS;
import static frc.robot.Constants.Swerve.MODULE_DRIVE_KP;
import static frc.robot.Constants.Swerve.MODULE_STEER_KD;
import static frc.robot.Constants.Swerve.MODULE_STEER_KP;
import static frc.robot.Constants.Swerve.STEER_DRIVE_BACKLASH;
import static frc.robot.Constants.Swerve.TICKS_TO_DEGREES;
import static frc.robot.Constants.Swerve.TICKS_TO_METERS;
import static frc.robot.Constants.Wiring.CANIVORE_BUS_ID;
import static frc.robot.Constants.Wiring.MODULE_CANCODER_IDS;
import static frc.robot.Constants.Wiring.MODULE_DRIVE_MOTOR_IDS;
import static frc.robot.Constants.Wiring.MODULE_STEER_MOTOR_IDS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private final int      id;
    private final TalonFX  driveMotor;
    private final TalonFX  steerMotor;
    private final CANCoder cancoder;

    /**
     * Constructs and configures a {@code SwerveModule} with the given ID.
     * Hardware CAN IDs and settings are read from {@link Constants}.
     *
     * @param moduleID
     */
    public SwerveModule(int moduleID) {
        id = moduleID;
        driveMotor = new TalonFX(MODULE_DRIVE_MOTOR_IDS[id], CANIVORE_BUS_ID);
        steerMotor = new TalonFX(MODULE_STEER_MOTOR_IDS[id], CANIVORE_BUS_ID);
        cancoder = new CANCoder(MODULE_CANCODER_IDS[id], CANIVORE_BUS_ID);

        new Thread(() -> {
            // Delay to ensure the Canivore has enough time to boot up
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            configCancoder();

            driveMotor.configFactoryDefault();
            driveMotor.setNeutralMode(NeutralMode.Brake);
            driveMotor.setInverted(true);
            driveMotor.setSelectedSensorPosition(0D);
            driveMotor.config_kP(0, MODULE_DRIVE_KP);
            // driveMotor.configClosedloopRamp(SWERVE_SECONDS_TO_FULL_SPEED);
            steerMotor.configFactoryDefault();
            steerMotor.setNeutralMode(NeutralMode.Brake);
            steerMotor.config_kP(0, MODULE_STEER_KP);
            steerMotor.config_kD(0, MODULE_STEER_KD);
            steerMotor.setSelectedSensorPosition(degreesToTicks(cancoder.getPosition() % 360));
        }).start();
    }

    private void configCancoder() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = 0;
        config.sensorDirection = false;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cancoder.configAllSettings(config);
        cancoder.setPosition(cancoder.getAbsolutePosition() - CANCODER_OFFSETS[id]);
    }

    /**
     * Sets the velocity of this modules's drive wheel
     *
     * @param metersPerSecond
     *        The desired velocity in meters per second
     */
    public void setVelocity(double metersPerSecond) {
        double ticksPer100ms = mpsToTicks100(metersPerSecond);
        double backlashRate = steerMotor.getSelectedSensorVelocity() * STEER_DRIVE_BACKLASH;
        driveMotor.set(TalonFXControlMode.Velocity, ticksPer100ms - backlashRate);
    }

    /**
     * @return the current velocity of this module's drive wheel in meters per
     *         second
     */
    public double getVelocity() {
        return ticks100ToMps(driveMotor.getSelectedSensorVelocity());
    }

    /**
     * @return the current encoder position of this module's drive wheel motor
     *         (used for odometry)
     */
    public double getDriveMotorPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    /**
     * Stops the current drive wheel from spinning. Effectively the same as
     * calling {@link #setVelocity(double) setVelocity(0)}
     */
    public void stopDriving() {
        setVelocity(0);
    }

    /**
     * Applies the given state to this module. This is the primary method of
     * commanding modules.
     *
     * @param state
     *        The desired state of the module
     */
    public void setState(SwerveModuleState state) {
        setSteerAngle(state.angle);
        setVelocity(state.speedMetersPerSecond);
    }

    /**
     * Sets the steer angle of the module, while clamping the movement to 180º.
     *
     * @param angle
     *        The angle to set
     */
    public void setSteerAngle(Rotation2d angle) {
        Rotation2d currentAngle = getSteerAngle();
        double diff = angle.minus(currentAngle)
                           .getDegrees();
        if (diff > 180) {
            diff -= 360;
        } else if (diff < -180) {
            diff += 360;
        }
        double newAngle = currentAngle.getDegrees() + diff;
        double ticks = degreesToTicks(newAngle);
        steerMotor.set(TalonFXControlMode.Position, ticks);
    }

    /**
     * @return An object containing the current angle and the net distance
     *         traveled by the module (used for odometry)
     */
    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(ticksToMeters(getDriveMotorPosition()), getSteerAngle());
    }

    /**
     * @return The net rotation this module has turned, unclamped.
     *
     * @see #getSteerAngleMod()
     */
    public Rotation2d getSteerAngle() {
        return Rotation2d.fromDegrees(ticksToDegrees(steerMotor.getSelectedSensorPosition()));
    }

    /**
     * @return The current direction this module's wheel is facing, clamped to
     *         the range (-π, π)
     */
    public Rotation2d getSteerAngleMod() {
        return getSteerAngle().plus(Rotation2d.fromDegrees(0));
    }

    /**
     * Converts from a module wheel's linear velocity to its drive motor's
     * angular velocity
     *
     * @param metersPerSecond
     *        the linear velocity of the wheel in meters per second
     *
     * @return the angular velocity of the motor in encoder ticks per 100
     *         milliseconds
     */
    private static double mpsToTicks100(double metersPerSecond) {
        return metersToTicks(metersPerSecond) * 0.1;
    }

    /**
     * Converts from a module's drive motor's angular velocity to its wheel's
     * linear velocity
     *
     * @param ticksPer100ms
     *        the angular velocuty of the module's drive motor in encoder ticks
     *        per 100ms
     *
     * @return the linear velocity of the wheel in meters per second
     */
    private static double ticks100ToMps(double ticksPer100ms) {
        return ticksToMeters(ticksPer100ms) * 10;
    }

    /**
     * Converts from a steer motor's encoder position to the wheel's heading
     *
     * @param ticks
     *        the motor's position in encoder ticks
     *
     * @return the heading of the wheel in degrees
     */
    private static double ticksToDegrees(double ticks) {
        return ticks * TICKS_TO_DEGREES;
    }

    /**
     * Converts from a wheel heading to its steer motor's encoder position
     *
     * @param degrees
     *        the heading of the wheel in degrees
     *
     * @return the motor's position in encoder ticks
     */
    private static double degreesToTicks(double degrees) {
        return degrees * DEGREES_TO_TICKS;
    }

    /**
     * Converts from a drive motor's encoder ticks to the net rotation of its
     * wheel
     *
     * @param ticks
     *        the motor encoder position in ticks
     *
     * @return the net linear distance traveled by the wheel in meters
     */
    private static double ticksToMeters(double ticks) {
        return ticks * TICKS_TO_METERS;
    }

    /**
     * Converts from a net wheel rotation to the net change in the drive motor's
     * encoder position
     *
     * @param meters
     *        the net linear distance traveled by the wheel in meters
     *
     * @return the motor encoder difference in ticks
     */
    private static double metersToTicks(double meters) {
        return meters * METERS_TO_TICKS;
    }
}
