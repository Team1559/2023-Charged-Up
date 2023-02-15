package frc.robot.subsystems.arm;

import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.Arm.MAXIMUM_ANGLE_ERROR;
import static frc.robot.Constants.Arm.kA_ELBOW;
import static frc.robot.Constants.Arm.kD_ELBOW;
import static frc.robot.Constants.Arm.kG_ELBOW;
import static frc.robot.Constants.Arm.kI_ELBOW;
import static frc.robot.Constants.Arm.kP_ELBOW;
import static frc.robot.Constants.Arm.kS_ELBOW;
import static frc.robot.Constants.Arm.kV_ELBOW;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmElbow extends SubsystemBase {
    // private final CANCoder canCoder;
    private final TalonFX  elbowMotor;
    private ArmFeedforward feedforward;
    private final double[] elbowPos = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    private int            degrees;

    public ArmElbow() {
        // canCoder = new CANCoder(ELBOW_CANCODER_ID);
        elbowMotor = new TalonFX(ARM_MOTOR_ID_ELBOW);
        elbowMotor.configFactoryDefault();
        elbowMotor.enableVoltageCompensation(true);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(
                true, 40, 60, 0.5);
        elbowMotor.configSupplyCurrentLimit(limit);
        elbowMotor.config_kP(0, kP_ELBOW);
        elbowMotor.config_kI(0, kI_ELBOW);
        elbowMotor.config_kD(0, kD_ELBOW);
        feedforward = new ArmFeedforward(kS_ELBOW, kG_ELBOW, kV_ELBOW,
                kA_ELBOW);
    }

    /**
     * private void configCancoder() { CANCoderConfiguration config = new
     * CANCoderConfiguration(); config.absoluteSensorRange =
     * AbsoluteSensorRange.Unsigned_0_to_360; config.magnetOffsetDegrees = 0;
     * config.sensorDirection = false; config.initializationStrategy =
     * SensorInitializationStrategy.BootToAbsolutePosition;
     * canCoder.configAllSettings(config); canCoder.setPosition(
     * canCoder.getAbsolutePosition() - ELBOW_CC_OFFSET); }
     */

    public double getAngle() {
        return elbowMotor.getSelectedSensorPosition();
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public void setAngle(double angle) {
        double FF = feedforward.calculate(angle, 0, 0);
        elbowMotor.set(TalonFXControlMode.Position, angleToTick(angle),
                DemandType.ArbitraryFeedForward, FF);
    }

    public int degreeSetUp() {
        degrees += 10;
        if (degrees > 90) {
            degrees = 90;
        }
        return degrees;
    }

    public int degreeSetDown() {
        degrees -= 10;
        if (degrees < 0) {
            degrees = 0;
        }
        return degrees;
    }

    public Command degreesUp() {
        return Commands.sequence(new InstantCommand(() -> degreeSetUp(), this),
                new InstantCommand(() -> setAngle(degrees), this));
    }

    public Command degreesDown() {
        return Commands.sequence(
                new InstantCommand(() -> degreeSetDown(), this),
                new InstantCommand(() -> setAngle(degrees), this));
    }

    public Command setElbowAngleCommandPos(int angleIndex) {
        double angle = elbowPos[angleIndex];
        return new FunctionalCommand(() -> {
            setAngle(angle);
        }, () -> {
        }, (a) -> {
        }, () -> {
            return isArmElbowAtPosition(angle);
        }, this);
    }

    public boolean isArmElbowAtPosition(double angle) {
        double angleError = Math.abs(angle - getAngle());
        if (angleError < MAXIMUM_ANGLE_ERROR) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elbow CANCoder reading (in deg): ",
                getAngle());
    }
}
