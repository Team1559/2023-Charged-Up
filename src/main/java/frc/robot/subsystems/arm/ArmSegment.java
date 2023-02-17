package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Arm.MAXIMUM_ANGLE_ERROR;
import static frc.robot.Constants.Arm.ANGULAR_VELOCITY_UNIT_TICKS;

public class ArmSegment extends SubsystemBase {
    private double        previousSetPoint;
    private final String  name;
    private final TalonFX motor;
    // private final CANCoder canCoder;
    private ArmFeedforward feedforward;
    private final double[] positions;
    private final double   gearReduction;
    private ArmSegment     previousSegment;
    private boolean        isSetPointCommanded = false;
    private double currentSetpoint;

    public ArmSegment(String name, int motorID, int cancoderID, double kp,
            double ki, double kd, double izone, double gearReduction,
            ArmFeedforward feedforward, double[] positions,
            ArmSegment previousSegment) {
        this.previousSegment = previousSegment;
        this.name = name;
        this.feedforward = feedforward;
        this.gearReduction = gearReduction;
        this.positions = positions;

        // canCoder = new CANCoder(cancoderID);
        motor = new TalonFX(motorID);
        motor.configFactoryDefault();
        motor.enableVoltageCompensation(true);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(
                true, 20, 80, 0.5);
        motor.configSupplyCurrentLimit(limit);
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
        motor.config_IntegralZone(0, izone);
        motor.configNeutralDeadband(0.005);
        motor.configClosedloopRamp(0.5);
    }

    public Command resetEncoderForTesting(double angle) {
        return new InstantCommand(
                () -> motor.setSelectedSensorPosition(angleToTick(angle)));
    }

    public double getAngle() {
        return tickToAngle(motor.getSelectedSensorPosition());
    }

    public double getGroundAngle() {
        double groundAnglePrevious = 0;
        if (previousSegment != null) {
            groundAnglePrevious = previousSegment.getGroundAngle();
        }
        double groundAngle = groundAnglePrevious + previousSetPoint;
        return groundAngle;
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

    public void setDestinationAngle(double angle) {
        currentSetpoint = getAngle();
        previousSetPoint = angle;
        isSetPointCommanded = true;
    }

    private void setAngleOnMotor(double angle) {
        double groundAngle = getGroundAngle();
        double FF = feedforward.calculate(groundAngle, 0, 0) / 12.0;
        motor.set(TalonFXControlMode.Position, angleToTick(angle),
                DemandType.ArbitraryFeedForward, FF);
        System.out.println("The method has been called. Setpoint = " + angle);
        SmartDashboard.putNumber(name + " Current motor setpoint: ", angle);
    }

    public Command setAngleCommandPos(int angleIndex) {
        double angle = positions[angleIndex];
        return new FunctionalCommand(() -> {
            setDestinationAngle(angle);
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
        SmartDashboard.putNumber(name + " motor temperature: ",
                motor.getTemperature());
        SmartDashboard.putNumber(name + " raw tick value: ",
                angleToTick(getAngle()));
        System.out.println("Setpoint Commanded = " + isSetPointCommanded);
        if (isSetPointCommanded == true) {
            System.out.println("in \"if\" statement, current angle = " + currentSetpoint);
            System.out.println("The previous stepoint is " + previousSetPoint);
            if (currentSetpoint < previousSetPoint - ANGULAR_VELOCITY_UNIT_TICKS) {
                System.out.println("commanding setpoint increase");
                currentSetpoint += ANGULAR_VELOCITY_UNIT_TICKS;
                setAngleOnMotor(currentSetpoint);
            } 
            if (currentSetpoint > previousSetPoint + ANGULAR_VELOCITY_UNIT_TICKS) {
                System.out.println("commanding setpoint decrease");
                currentSetpoint -= ANGULAR_VELOCITY_UNIT_TICKS;
                setAngleOnMotor(currentSetpoint);
            }
        } else {
            isSetPointCommanded = false;
        }
        SmartDashboard.putNumber(name + " previous commanded angle ", previousSetPoint);
        SmartDashboard.putBoolean(name + " \"is setpoint commanded\": ", isSetPointCommanded);
    }
}
