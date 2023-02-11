package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Arm.MAXIMUM_ANGLE_ERROR;

public class ArmSegment extends SubsystemBase {

    private final String  name;
    private final TalonFX motor;
    // private final CANCoder canCoder;
    private final ArmFeedforward feedforward;
    private final double[]       positions;
    private final double         gearReduction;

    public ArmSegment(String name, int motorID, int cancoderID, double kp,
            double ki, double kd, double izone, double gearReduction,
            ArmFeedforward feedforward, double[] positions) {

        this.name = name;
        this.feedforward = feedforward;
        this.gearReduction = gearReduction;
        this.positions = positions;

        // canCoder = new CANCoder(cancoderID);
        motor = new TalonFX(motorID);
        motor.configFactoryDefault();
        motor.enableVoltageCompensation(true);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(
                true, 20, 25, 0.5);
        motor.configSupplyCurrentLimit(limit);
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
        motor.config_IntegralZone(0, izone);
        motor.setSelectedSensorPosition(angleToTick(90));
    }

    public void resetEncoderForTesting(double angle) {
        motor.setSelectedSensorPosition(angleToTick(angle));
    }

    public double getAngle() {
        return tickToAngle(motor.getSelectedSensorPosition());
    }

    public double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * gearReduction;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public double tickToAngle(double ticks) {
        double motorRevolutions = ticks / FALCON_TICKS_PER_REV;
        double revolutionsOfArm = motorRevolutions / gearReduction;
        double angle = revolutionsOfArm * 360;
        return angle;
    }

    public void setAngle(double angle) {
        double FF = feedforward.calculate(angle, 0, 0) / 12.0;
        motor.set(TalonFXControlMode.Position, angleToTick(angle),
                DemandType.ArbitraryFeedForward, FF);
    }

    public Command setAngleCommandPos(int angleIndex) {
        double angle = positions[angleIndex];
        return new FunctionalCommand(() -> {
            setAngle(angle);
        }, () -> {
        }, (a) -> {
        }, () -> {
            return IsAtPosition(angle);
        }, this);
    }

    public boolean IsAtPosition(double angle) {
        double angleError = Math.abs(angle - getAngle());
        if (angleError < MAXIMUM_ANGLE_ERROR) {
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(name + " angle: ", getAngle());
        SmartDashboard.putNumber(name + " current: ", motor.getStatorCurrent());
    }
}
