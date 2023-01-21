package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.CANCODER_OFFSETS;
import static frc.robot.Constants.Swerve.DEGREES_TO_TICKS;
import static frc.robot.Constants.Swerve.METERS_TO_TICKS;
import static frc.robot.Constants.Swerve.TICKS_TO_DEGREES;
import static frc.robot.Constants.Swerve.TICKS_TO_METERS;
import static frc.robot.Constants.Wiring.MODULE_CANCODER_IDS;
import static frc.robot.Constants.Wiring.MODULE_DRIVE_MOTOR_IDS;
import static frc.robot.Constants.Wiring.MODULE_STEER_MOTOR_IDS;

import java.util.function.DoubleConsumer;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final int      id;
    private final TalonFX  driveMotor;
    private final TalonFX  steerMotor;
    private final CANCoder cancoder;

    public SwerveModule(int moduleID) {
        id = moduleID;
        driveMotor = new TalonFX(MODULE_DRIVE_MOTOR_IDS[id]);
        steerMotor = new TalonFX(MODULE_STEER_MOTOR_IDS[id]);
        cancoder = new CANCoder(MODULE_CANCODER_IDS[id]);

        configCancoder();

        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(true);
        driveMotor.setSelectedSensorPosition(0D);
        driveMotor.config_kP(0, 0.05);

        steerMotor.configFactoryDefault();
        steerMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.config_kP(0, 0.22);
        steerMotor.config_kD(0, 0.1);
        steerMotor.setSelectedSensorPosition(
                degreesToTicks(cancoder.getPosition() % 360));
    }

    private void configCancoder() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = 0;
        config.sensorDirection = false;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        cancoder.configAllSettings(config);
        cancoder.setPosition(
                cancoder.getAbsolutePosition() - CANCODER_OFFSETS[id]);
    }

    public void setVelocity(double metersPerSecond) {
        double ticksPer100ms = mpsToTicks100(metersPerSecond);
        SmartDashboard.putNumber("Velocity ticks", ticksPer100ms);
        driveMotor.set(TalonFXControlMode.Velocity, ticksPer100ms);
    }

    public double getVelocity() {
        return ticks100ToMps(driveMotor.getSelectedSensorVelocity());
    }

    public double getDriveMotorPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public void stopDriving() {
        setVelocity(0);
    }

    public void setState(SwerveModuleState state) {
        setSteerAngle(state.angle);
        setVelocity(state.speedMetersPerSecond);
    }

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
        if (id == 0) {
            SmartDashboard.putNumber("Input Angle", angle.getDegrees());
            SmartDashboard.putNumber("Adjusted angle", newAngle);
            SmartDashboard.putNumber("Calculated value", ticks);
            SmartDashboard.putNumber("Current position",
                    steerMotor.getSelectedSensorPosition());

            SmartDashboard.putNumber("Current position degrees",
                    ticksToDegrees(steerMotor.getSelectedSensorPosition()));
        }
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(
                ticksToMeters(driveMotor.getSelectedSensorPosition()),
                getSteerAngle());
    }

    public Rotation2d getSteerAngle() {
        return Rotation2d.fromDegrees(
                ticksToDegrees(steerMotor.getSelectedSensorPosition()));
    }

    public Rotation2d getSteerAngleMod() {
        return getSteerAngle().plus(Rotation2d.fromDegrees(0));
    }

    private static double mpsToTicks100(double metersPerSecond) {
        return metersToTicks(metersPerSecond) * 0.1;
    }

    private static double ticks100ToMps(double ticksPer100ms) {
        return ticksToMeters(ticksPer100ms) * 10;
    }

    private static double ticksToDegrees(double ticks) {
        return ticks * TICKS_TO_DEGREES;
    }

    private static double degreesToTicks(double degrees) {
        return degrees * DEGREES_TO_TICKS;
    }

    private static double ticksToMeters(double ticks) {
        return ticks * TICKS_TO_METERS;
    }

    private static double metersToTicks(double meters) {
        return meters * METERS_TO_TICKS;
    }
}
