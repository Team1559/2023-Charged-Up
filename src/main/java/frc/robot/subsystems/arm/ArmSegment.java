package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSegment extends SubsystemBase {
    private final String name;
    private ArmSegment   lowerSegment;
    private ArmSegment   higherSegment;

    private final TalonFX  motor;
    private final CANCoder canCoder;

    private final Translation2d centerOfMass;
    private final double        mass;
    private final double        length;
    private final double        stallTorque;
    private final double        gearRatio;
    private final double        efficiency;
    private final double        maxSpeed;
    private final double        acceleration;

    private final double[] positions;
    private double         target;
    private boolean        isSetPointCommanded = false;
    private double         setpointJointAngle;
    private double         speed;
    private double         stopAccelPoint;
    private double         decelPoint;

    public ArmSegment(String name, int motorID, int cancoderID, double kp, double ki, double kd,
            double izone, double gearRatio, double[] positions, double efficiency,
            double maxVelocity, double acceleration, double mass, double length,
            Translation2d centerOfMass, boolean isInverted, double lowerLimit, double upperLimit,
            double closedLoopErrorValue) {
        this.name = name;
        this.gearRatio = gearRatio;
        this.positions = positions;
        this.maxSpeed = maxVelocity;
        this.acceleration = acceleration;
        this.efficiency = efficiency;
        this.mass = mass;
        this.length = length;
        this.centerOfMass = centerOfMass;

        this.stallTorque = gearRatio * FALCON_STALL_TORQUE;
        // if (isInverted) {
        // kd *= -1;
        // kp *= -1;
        // ki *= -1;
        // }
        motor = new TalonFX(motorID);
        motor.configFactoryDefault();
        motor.enableVoltageCompensation(true);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 20, 80,
                0.5);
        motor.configSupplyCurrentLimit(limit);
        motor.config_kP(0, kp);
        motor.config_kI(0, ki);
        motor.config_kD(0, kd);
        motor.config_IntegralZone(0, izone);
        motor.configNeutralDeadband(0.001);
        motor.configClosedloopRamp(0.5);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configClosedLoopPeakOutput(0, 0.2);
        canCoder = new CANCoder(cancoderID);
        configCancoder(canCoder);
        // Configure the CanCoder to be remote sensor 0,
        // then select remote sensor 0 as our PID input.
        motor.configRemoteFeedbackFilter(canCoder, 0);
        motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        motor.setSensorPhase(isInverted);
        motor.setInverted(isInverted);
        // motor.configAllowableClosedloopError(0, closedLoopErrorValue);

        // if (isInverted) {
        // motor.configSelectedFeedbackCoefficient(-1);
        // Motor is mounted in opposite orientation (inverted)
        // }
        // motor.setInverted(isInverted);

        motor.configForwardSoftLimitEnable(true); // will eventually enable
        motor.configReverseSoftLimitEnable(true);
        motor.configForwardSoftLimitThreshold(angleToTick(upperLimit));
        motor.configReverseSoftLimitThreshold(angleToTick(lowerLimit));
    }

    private void configCancoder(CANCoder canCoder) {
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        // setting to true b/c CANcoders are on opposite side of robot
        canCoder.configSensorDirection(true);
        canCoder.configSensorInitializationStrategy(
                SensorInitializationStrategy.BootToAbsolutePosition);
        canCoder.setPosition(canCoder.getAbsolutePosition());
    }

    public double getJointAngle() {
        return tickToAngle(motor.getSelectedSensorPosition());
    }

    public double getSetpointGroundAngle() {
        if (lowerSegment == null) {
            return setpointJointAngle;
        }
        // A ground angle is the angle of the previous segment, added to the
        // angle of the current segment.
        return setpointJointAngle + lowerSegment.getSetpointGroundAngle();
    }

    public double getMass() {
        return mass;
    }

    public double getHigherMass() {
        return higherSegment == null ? mass : (mass + higherSegment.getHigherMass());
    }

    public Translation2d getRelativeEndpoint() {
        return new Translation2d(length, Rotation2d.fromDegrees(getSetpointGroundAngle()));
    }

    public Translation2d getRelativeCenterOfMass() {
        Rotation2d pivotPosition = Rotation2d.fromDegrees(getSetpointGroundAngle());
        if (higherSegment == null) {
            return centerOfMass.rotateBy(pivotPosition);
        }

        double higherMass = higherSegment.getHigherMass();
        Translation2d higherCenterOfMass = higherSegment.getRelativeCenterOfMass()
                                                        .plus(getRelativeEndpoint());
        Translation2d scaledHigherCoM = higherCenterOfMass.times(higherMass);
        Translation2d scaledMyCoM = centerOfMass.rotateBy(pivotPosition)
                                                .times(mass);
        Translation2d centerOfMassCalc = scaledHigherCoM.plus(scaledMyCoM)
                                                        .div(higherMass + mass);
        return centerOfMassCalc;
    }

    public double calculateKG(Translation2d totalCenterOfMass) {
        double torqueRequired = getHigherMass() * GRAVITY_ACCELERATION
                * totalCenterOfMass.getNorm();
        return torqueRequired / stallTorque * efficiency;
    }

    public double calculateKA(Translation2d totalCenterOfMass) {
        return getHigherMass() * totalCenterOfMass.getNorm() / stallTorque * (Math.PI / 180);
    }

    public double calculateKV(Translation2d totalCenterOfMass) {
        return 1 / (FALCON_MAX_RPM * 360 / 60) / totalCenterOfMass.getNorm();
    }

    public double calculateFeedForward(double velocity, double acceleration) {
        /**
         * int inversionCoefficient = 0; if (isInverted) { inversionCoefficient
         * = -1; } else { inversionCoefficient = 1; }
         */
        Translation2d totalCenterOfMass = getRelativeCenterOfMass();
        double kG = calculateKG(totalCenterOfMass);
        double kV = calculateKV(totalCenterOfMass);
        double kA = calculateKA(totalCenterOfMass);
        return (kV * velocity + kA * acceleration + kG * totalCenterOfMass.getAngle()
                                                                          .getCos());
        // * inversionCoefficient;
    }

    public void setLowerSegment(ArmSegment lowerSegment) {
        this.lowerSegment = lowerSegment;
    }

    public void setHigherSegment(ArmSegment higherSegment) {
        this.higherSegment = higherSegment;
    }

    public double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        return revolutionsOfArm * CANCODER_TICKS_PER_REV;
    }

    public double tickToAngle(double ticks) {
        double revolutionsOfArm = ticks / CANCODER_TICKS_PER_REV;
        double angle = revolutionsOfArm * 360;
        return angle;
    }

    public void setDestinationJointAngle(double destinationJointAngle) {
        target = destinationJointAngle;
        setpointJointAngle = getJointAngle();
        isSetPointCommanded = true;

        double displacementToSetpoint = destinationJointAngle - setpointJointAngle;
        double accelerationDisplacement = Math.copySign(0.5 * maxSpeed * maxSpeed / acceleration,
                displacementToSetpoint);
        if (Math.abs(accelerationDisplacement) > Math.abs(displacementToSetpoint / 2)) {
            accelerationDisplacement = displacementToSetpoint / 2;
        }
        stopAccelPoint = setpointJointAngle + accelerationDisplacement;
        decelPoint = target - accelerationDisplacement;
    }

    private void setJointAngleOnMotor(double angle, double velocity, double acceleration) {
        double FF = calculateFeedForward(velocity, acceleration);
        motor.set(TalonFXControlMode.Position, angleToTick(angle), DemandType.ArbitraryFeedForward,
                FF);
    }

    public Command setAngleCommandPos(int angleIndex) {
        double angle = positions[angleIndex];
        return new FunctionalCommand(() -> {
            setDestinationJointAngle(angle);
        }, () -> {
        }, (a) -> {
        }, () -> {
            return IsAtPosition(angle);
        }, this);
    }

    public boolean IsAtPosition(double jointAngle) {
        double angleError = Math.abs(jointAngle - getJointAngle());
        return angleError < MAXIMUM_ANGLE_ERROR;
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleopEnabled()) {
            isSetPointCommanded = false;
            setpointJointAngle = getJointAngle();
            target = setpointJointAngle;
        }
        double accel = 0;
        double velo = 0;
        if (isSetPointCommanded) {
            double distanceToTarget = Math.abs(setpointJointAngle - target);
            if (distanceToTarget <= Math.max(speed, MINIMUM_TARGET_DISTANCE)) {
                setpointJointAngle = target;
                speed = 0;
            } else {
                if (setpointJointAngle > target) {
                    // Need to swap comparisons if moving in reverse
                    if (setpointJointAngle <= decelPoint) {
                        accel = -acceleration;
                        speed -= acceleration;
                    } else if (setpointJointAngle > stopAccelPoint) {
                        accel = acceleration;
                        speed += acceleration;
                    }
                } else {
                    if (setpointJointAngle >= decelPoint) {
                        accel = -acceleration;
                        speed -= acceleration;
                    } else if (setpointJointAngle < stopAccelPoint) {
                        accel = acceleration;
                        speed += acceleration;
                    }
                }

                if (speed < maxSpeed * 0.05) {
                    speed = maxSpeed * 0.05;
                }

                double nextSetpoint = setpointJointAngle;
                if (setpointJointAngle > target) {
                    velo = -speed;
                    nextSetpoint -= speed;
                } else {
                    velo = speed;
                    nextSetpoint += speed;
                }
                if (nextSetpoint >= target != setpointJointAngle >= target) {
                    setpointJointAngle = target;
                } else {
                    setpointJointAngle = nextSetpoint;
                }
            }

            setJointAngleOnMotor(setpointJointAngle, velo * CYCLES_PER_SECOND,
                    accel * CYCLES_PER_SECOND * CYCLES_PER_SECOND);
        } else {
            motor.neutralOutput();
        }

        Translation2d centerOfMass = getRelativeCenterOfMass();
        SmartDashboard.putString(name + " Center of mass: ",
                String.format(formatPolar(centerOfMass)));
        SmartDashboard.putNumber(name + " kG: ", calculateKG(centerOfMass));
        SmartDashboard.putNumber(name + " kV: ", calculateKV(centerOfMass));
        SmartDashboard.putNumber(name + " kA: ", calculateKA(centerOfMass));
        SmartDashboard.putNumber(name + " angle: ", getJointAngle());
        SmartDashboard.putNumber(name + "CANCoder Angle: ", canCoder.getAbsolutePosition());
        SmartDashboard.putNumber(name + "CANCoder Relative: ", canCoder.getPosition());
        SmartDashboard.putNumber(name + " setpoint: ", setpointJointAngle);
        SmartDashboard.putNumber(name + " ff",
                calculateFeedForward(velo * CYCLES_PER_SECOND, accel * CYCLES_PER_SECOND));
        SmartDashboard.putNumber(name + " current draw:", motor.getSupplyCurrent());
        SmartDashboard.putNumber(name + " error: ", motor.getClosedLoopError());
        SmartDashboard.putNumber(name + " speed: ", speed);
    }

    private static String formatPolar(Translation2d t) {
        return String.format("ø=%4.1f, m=%4.2f", t.getAngle()
                                                  .getDegrees(),
                t.getNorm());
    }
}
