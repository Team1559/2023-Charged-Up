package frc.robot.subsystems.arm;

import static frc.robot.Constants.CYCLES_PER_SECOND;
import static frc.robot.Constants.FALCON_MAX_RPM;
import static frc.robot.Constants.FALCON_STALL_TORQUE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.GRAVITY_ACCELERATION;
import static frc.robot.Constants.Arm.MAXIMUM_ANGLE_ERROR;
import static frc.robot.Constants.Arm.MINIMUM_TARGET_DISTANCE;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSegment extends SubsystemBase {
    private final String name;
    private ArmSegment   lowerSegment;
    private ArmSegment   higherSegment;

    private final TalonFX motor;
    // private final CANCoder canCoder;

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
    private double         setpoint;
    private double         speed;
    private double         stopAccelPoint;
    private double         decelPoint;

    public ArmSegment(String name, int motorID, int cancoderID, double kp, double ki, double kd,
            double izone, double gearRatio, double[] positions, double efficiency,
            double maxVelocity, double acceleration, double mass, double length,
            Translation2d centerOfMass) {
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

        // canCoder = new CANCoder(cancoderID);
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
        motor.setNeutralMode(NeutralMode.Coast);
    }

    // private void configCancoder() {
    // CANCoderConfiguration config = new CANCoderConfiguration();
    // config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // config.magnetOffsetDegrees = 0;
    // config.sensorDirection = false;
    // config.initializationStrategy =
    // SensorInitializationStrategy.BootToAbsolutePosition;
    // canCoder.configAllSettings(config);
    // canCoder.setPosition(canCoder.getAbsolutePosition() - BASE_CC_OFFSET);
    // }

    public Command resetEncoderForTesting(double angle) {
        return new InstantCommand(() -> motor.setSelectedSensorPosition(angleToTick(angle)));
    }

    public double getAngle() {
        return tickToAngle(motor.getSelectedSensorPosition());
    }

    public double getGroundAngle() {
        return setpoint;
    }

    public double getMass() {
        return mass;
    }

    public double getHigherMass() {
        return higherSegment == null ? mass : (mass + higherSegment.getHigherMass());
    }

    public Translation2d getRelativeEndpoint() {
        return new Translation2d(length, Rotation2d.fromDegrees(setpoint));
    }

    public Translation2d getRelativeCenterOfMass() {
        Rotation2d pivotPosition = Rotation2d.fromDegrees(setpoint);
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
        Translation2d totalCenterOfMass = getRelativeCenterOfMass();
        double kG = calculateKG(totalCenterOfMass);
        double kV = calculateKV(totalCenterOfMass);
        double kA = calculateKA(totalCenterOfMass);
        return kV * velocity + kA * acceleration + kG * totalCenterOfMass.getAngle()
                                                                         .getCos();
        // return kG * totalCenterOfMass.getAngle()
        // .getCos();
    }

    public void setLowerSegment(ArmSegment lowerSegment) {
        this.lowerSegment = lowerSegment;
    }

    public void setHigherSegment(ArmSegment higherSegment) {
        this.higherSegment = higherSegment;
    }

    public double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * gearRatio;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public double tickToAngle(double ticks) {
        double motorRevolutions = ticks / FALCON_TICKS_PER_REV;
        double revolutionsOfArm = motorRevolutions / gearRatio;
        double angle = revolutionsOfArm * 360;
        return angle;
    }

    public void setDestinationAngle(double angle) {
        target = angle;
        setpoint = getAngle();
        isSetPointCommanded = true;

        double displacementToSetpoint = angle - setpoint;
        double accelerationDisplacement = Math.copySign(0.5 * maxSpeed * maxSpeed / acceleration,
                displacementToSetpoint);
        stopAccelPoint = setpoint + accelerationDisplacement;
        decelPoint = target - accelerationDisplacement;
    }

    private void setAngleOnMotor(double angle, double velocity, double acceleration) {
        double FF = calculateFeedForward(velocity, acceleration);
        motor.set(TalonFXControlMode.Position, angleToTick(angle), DemandType.ArbitraryFeedForward,
                FF);
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
        return angleError < MAXIMUM_ANGLE_ERROR;
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleopEnabled()) {
            isSetPointCommanded = false;
            setpoint = getAngle();
            target = getAngle();
        }
        double accel = 0;
        double velo = 0;
        if (isSetPointCommanded) {
            double distanceToTarget = Math.abs(setpoint - target);
            if (distanceToTarget <= Math.max(speed, MINIMUM_TARGET_DISTANCE)) {
                setpoint = target;
                speed = 0;
            } else {
                if (setpoint > target) {
                    // Need to swap comparisons if moving in reverse
                    if (setpoint <= decelPoint) {
                        accel = -acceleration;
                        speed -= acceleration;
                    } else if (setpoint > stopAccelPoint) {
                        accel = acceleration;
                        speed += acceleration;
                    }
                } else {
                    if (setpoint >= decelPoint) {
                        accel = -acceleration;
                        speed -= acceleration;
                    } else if (setpoint < stopAccelPoint) {
                        accel = acceleration;
                        speed += acceleration;
                    }
                }

                if (speed < maxSpeed * 0.05) {
                    speed = maxSpeed * 0.05;
                }

                double nextSetpoint = setpoint;
                if (setpoint > target) {
                    velo = -speed;
                    nextSetpoint -= speed;
                } else {
                    velo = speed;
                    nextSetpoint += speed;
                }
                if (nextSetpoint >= target != setpoint >= target) {
                    setpoint = target;
                } else {
                    setpoint = nextSetpoint;
                }
            }

            setAngleOnMotor(setpoint, velo * CYCLES_PER_SECOND,
                    accel * CYCLES_PER_SECOND * CYCLES_PER_SECOND);
        } else {
            motor.neutralOutput();
        }

        // Translation2d centerOfMass = getRelativeCenterOfMass();
        // SmartDashboard.putNumber(name + " kG: ", calculateKG(centerOfMass));
        // SmartDashboard.putNumber(name + " kV: ", calculateKV(centerOfMass));
        // SmartDashboard.putNumber(name + " kA: ", calculateKA(centerOfMass));
        // SmartDashboard.putNumber(name + " angle: ", getAngle());
        // SmartDashboard.putNumber(name + " setpoint: ", setpoint);
        // SmartDashboard.putNumber(name + " ff",
        // calculateFeedForward(velo * CYCLES_PER_SECOND, accel *
        // CYCLES_PER_SECOND));
        // SmartDashboard.putNumber(name + " current draw:",
        // motor.getSupplyCurrent());
        // SmartDashboard.putNumber(name + " error: ",
        // motor.getClosedLoopError());
        // Translation2d com = getRelativeCenterOfMass();
        // SmartDashboard.putString(name + " Center of mass: ",
        // String.format(formatPolar(com)));
    }

    private static String formatPolar(Translation2d t) {
        return String.format("ø=%4.1f, m=%4.2f", t.getAngle()
                                                  .getDegrees(),
                t.getNorm());
    }
}
