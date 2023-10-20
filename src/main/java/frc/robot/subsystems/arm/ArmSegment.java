package frc.robot.subsystems.arm;

import static frc.robot.Constants.CANCODER_TICKS_PER_REV;
import static frc.robot.Constants.CYCLES_PER_SECOND;
import static frc.robot.Constants.FALCON_MAX_RPM;
import static frc.robot.Constants.FALCON_STALL_TORQUE;
import static frc.robot.Constants.GRAVITY_ACCELERATION;
import static frc.robot.Constants.Arm.MAXIMUM_ANGLE_ERROR;
import static frc.robot.Constants.Arm.MINIMUM_TARGET_DISTANCE;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ArmSegment extends SubsystemBase {
    private final String name;
    private ArmSegment   lowerSegment;
    private ArmSegment   higherSegment;

    private final TalonFX  motor;
    private final CANCoder canCoder;
    private Pigeon2        imu;

    private final Translation2d centerOfMass;
    private final double        mass;
    private final double        length;
    private final double        stallTorque;
    private final double        efficiency;
    private final double        maxSpeed;
    private final double        acceleration;
    private final double        deceleration;
    private final double        defaultKP;
    private final double        cancoderOffset;

    private Arm.Position targetPosition;
    private Arm.Position lastPosition;
    private double       target;
    private double       accel;
    private boolean      isSetPointCommanded = false;
    private double       setpointJointAngle;
    private double       speed;
    private double       stopAccelPoint;
    private double       decelPoint;
    private MedianFilter accelFilter;

    public ArmSegment(String name, int motorID, int cancoderID, Pigeon2 imu, double kp, double ki,
            double kd, double izone, double gearRatio, double efficiency, double maxVelocity,
            double acceleration, double deceleration, double mass, double length,
            Translation2d centerOfMass, boolean isInverted, double lowerLimit, double upperLimit,
            double closedLoopErrorValue, double cancoderOffset) {
        this.cancoderOffset = cancoderOffset;
        this.name = name;
        this.maxSpeed = maxVelocity;
        this.acceleration = acceleration;
        this.deceleration = deceleration;
        this.efficiency = efficiency;
        this.mass = mass;
        this.length = length;
        this.centerOfMass = centerOfMass;
        this.stallTorque = gearRatio * FALCON_STALL_TORQUE;
        this.imu = imu;
        this.accelFilter = new MedianFilter(5);
        this.defaultKP = kp;

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

        motor.configForwardSoftLimitEnable(true);
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
        canCoder.setPosition(canCoder.getAbsolutePosition() + cancoderOffset);
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

    public double calculateAccelCompensation(Translation2d totalCenterOfMass) {
        short[] accels = new short[3];
        imu.getBiasedAccelerometer(accels);
        double accel = accels[0] / 16384.0 * GRAVITY_ACCELERATION;
        double filteredAccel = accelFilter.calculate(accel);
        SmartDashboard.putNumber(name + "AccelX", accel);
        double torqueRequired = -filteredAccel * getHigherMass() * totalCenterOfMass.getNorm()
                * totalCenterOfMass.getAngle()
                                   .getSin();
        SmartDashboard.putNumber(name + "AccelCompTorque", torqueRequired);
        return torqueRequired * 1.0 / stallTorque;
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
        if (name.equalsIgnoreCase("base") && setpointJointAngle >= 90) {
            return 0.0;
        }
        Translation2d totalCenterOfMass = getRelativeCenterOfMass();
        double kG = calculateKG(totalCenterOfMass);
        double kV = calculateKV(totalCenterOfMass);
        double kA = calculateKA(totalCenterOfMass);
        double accelComp = calculateAccelCompensation(totalCenterOfMass);
        accelComp = 0;
        return (accelComp + kV * velocity + kA * acceleration + kG * totalCenterOfMass.getAngle()
                                                                                      .getCos());
    }

    public Arm.Position getTargetPosition() {
        return targetPosition;
    }

    public Arm.Position getLastPosition() {
        return lastPosition;
    }

    public boolean lastPosEqualsTarget() {
        return targetPosition == lastPosition && targetPosition != null;
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

    public double getBaseAccel() {
        if (name.equalsIgnoreCase("elbow")) {
            return lowerSegment.getAccel();
        } else {
            return 0;
        }
    }

    public double getAccel() {
        return accel;
    }

    public void setDestinationJointAngle(Arm.Position position) {
        double destinationJointAngle = getTargetAngle(position);
        target = destinationJointAngle;
        setpointJointAngle = getJointAngle();
        isSetPointCommanded = true;

        double displacementToSetpoint = destinationJointAngle - setpointJointAngle;
        double accelerationDisplacement = Math.copySign(0.5 * maxSpeed * maxSpeed / acceleration,
                displacementToSetpoint);
        double decelerationDisplacement = Math.copySign(0.5 * maxSpeed * maxSpeed / deceleration,
                displacementToSetpoint);
        double accelDecelDisplacement = accelerationDisplacement + decelerationDisplacement;

        if (Math.abs(accelDecelDisplacement) > Math.abs(displacementToSetpoint)) {
            double scaleFactor = displacementToSetpoint / accelDecelDisplacement;
            accelerationDisplacement *= scaleFactor;
            decelerationDisplacement *= scaleFactor;
        }
        stopAccelPoint = setpointJointAngle + accelerationDisplacement;
        decelPoint = target - decelerationDisplacement;
    }

    private void setJointAngleOnMotor(double angle, double velocity, double acceleration) {
        if (tickToAngle(motor.getClosedLoopError()) > 2 && name.equalsIgnoreCase("elbow")
                && (targetPosition == Arm.Position.PICKUP_CONE
                        || targetPosition == Arm.Position.PICKUP_CUBE)) {
            motor.config_kP(0, defaultKP * 1.5);
        } else if (name.equalsIgnoreCase("elbow")) {
            motor.config_kP(0, defaultKP);
        }
        if (angle >= 100 && name.equalsIgnoreCase("base")) {
            motor.set(TalonFXControlMode.PercentOutput, 0.10);
        } else {
            double FF = calculateFeedForward(velocity, acceleration);
            motor.set(TalonFXControlMode.Position, angleToTick(angle),
                    DemandType.ArbitraryFeedForward, FF);
        }
    }

    public void armPanic() {
        motor.neutralOutput();
        forgetEverything();
    }

    protected abstract double getTargetAngle(Arm.Position position);

    public Command setAngleCommandPos(Arm.Position position) {
        return new ArmSegmentPositionCommand(position);
    }

    public boolean isAtPosition(Arm.Position position) {
        double maxError = MAXIMUM_ANGLE_ERROR;
        if (tickToAngle(motor.getSelectedSensorVelocity()) < .3) {
            maxError *= 10;
        }
        double jointAngle = getTargetAngle(position);
        double angleError = Math.abs(jointAngle - getJointAngle());
        boolean isAtPosition = angleError < maxError;
        if (isAtPosition) {
            lastPosition = targetPosition;
        }
        return isAtPosition;
    }

    private void forgetEverything() {
        isSetPointCommanded = false;
        setpointJointAngle = getJointAngle();
        target = setpointJointAngle;
        speed = 0;
        accel = 0;
    }

    public void increaseKP() {
        // motor.config_kP(0, 1.5 * defaultKP);
    }

    public void resetKP() {
        // int count = 0;
        // while (motor.config_kP(0, defaultKP) != ErrorCode.OK && count < 50) {
        // count++;
        // try {
        // Thread.sleep(1);
        // } catch (InterruptedException e) {
        // // ignore
        // }
        // }
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            forgetEverything();
        }
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
                        accel = -deceleration;
                        speed -= deceleration;
                    } else if (setpointJointAngle > stopAccelPoint) {
                        accel = acceleration;
                        speed += acceleration;
                    }
                } else {
                    if (setpointJointAngle >= decelPoint) {
                        accel = -deceleration;
                        speed -= deceleration;
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
                    (accel + this.getBaseAccel()) * CYCLES_PER_SECOND * CYCLES_PER_SECOND);
        } else {
            motor.neutralOutput();
        }

        SmartDashboard.putNumber(name + " angle: ", getJointAngle());
        SmartDashboard.putNumber(name + " setpoint: ", setpointJointAngle);
        SmartDashboard.putNumber(name + " error", motor.getClosedLoopError());
        SmartDashboard.putNumber(name + " current", motor.getSupplyCurrent());
    }

    private class ArmSegmentPositionCommand extends CommandBase {
        private final Arm.Position destinationPos;

        ArmSegmentPositionCommand(Arm.Position position) {
            destinationPos = position;
            addRequirements(ArmSegment.this);
        }

        @Override
        public void initialize() {
            setDestinationJointAngle(destinationPos);
        }

        @Override
        public boolean isFinished() {
            return isAtPosition(destinationPos);
        }

        @Override
        public void end(boolean interrupted) {
            SmartDashboard.putNumber(name + " finish", getJointAngle());
        }
    }
}
