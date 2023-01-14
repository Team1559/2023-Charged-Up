package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.CANCODER_OFFSETS;
import static frc.robot.Constants.Swerve.DEGREES_TO_TICKS;
import static frc.robot.Constants.Swerve.METERS_TO_TICKS;
import static frc.robot.Constants.Swerve.TICKS_TO_DEGREES;
import static frc.robot.Constants.Swerve.TICKS_TO_METERS;
import static frc.robot.Constants.Wiring.MODULE_CANCODER_IDS;
import static frc.robot.Constants.Wiring.MODULE_DRIVE_MOTOR_IDS;
import static frc.robot.Constants.Wiring.MODULE_STEER_MOTOR_IDS;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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

        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(true);
        steerMotor.setNeutralMode(NeutralMode.Coast);
        steerMotor.setInverted(true);

        driveMotor.setSelectedSensorPosition(0D);
        cancoder.configMagnetOffset(CANCODER_OFFSETS[id]);
        steerMotor.setSelectedSensorPosition(
                degreesToTicks(cancoder.getPosition() % 360));
    }

    public void setVelocity(double metersPerSecond) {
        driveMotor.config_kP(0, SmartDashboard.getNumber("kP", -1));
        double ticksPer100ms = mpsToTicks100(metersPerSecond);
        driveMotor.set(TalonFXControlMode.Velocity, ticksPer100ms);
        SmartDashboard.putNumber("CANCoder Pos " + id,
                cancoder.getAbsolutePosition());
        SmartDashboard.putNumber("CANCoder Pos O " + id,
                cancoder.getPosition());
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
        steerMotor.set(TalonFXControlMode.Position,
                degreesToTicks(angle.getDegrees()));
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

    private static double mpsToTicks100(double metersPerSecond) {
        return metersToTicks(metersPerSecond) * 10;
    }

    private static double ticks100ToMps(double ticksPer100ms) {
        return ticksToMeters(ticksPer100ms) * 0.1;
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
