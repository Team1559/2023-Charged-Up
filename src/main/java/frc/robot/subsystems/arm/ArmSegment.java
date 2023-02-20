package frc.robot.subsystems.arm;

import static frc.robot.Constants.FALCON_STALL_TORQUE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.GRAVITY_ACCELERATION;
import static frc.robot.Constants.NOMINAL_VOLTAGE;
import static frc.robot.Constants.Arm.ANGULAR_VELOCITY_UNIT_TICKS;
import static frc.robot.Constants.Arm.MAXIMUM_ANGLE_ERROR;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final double[] positions;
    private double         setpoint;
    private boolean        isSetPointCommanded = false;
    private double         interpolatedSetpoint;

    public ArmSegment(String name, int motorID, int cancoderID, double kp,
            double ki, double kd, double izone, double gearRatio,
            double[] positions, double efficiency, double mass, double length,
            Translation2d centerOfMass) {
        this.name = name;
        this.gearRatio = gearRatio;
        this.positions = positions;

        this.efficiency = efficiency;
        this.mass = mass;
        this.length = length;
        this.centerOfMass = centerOfMass;

        this.stallTorque = gearRatio * FALCON_STALL_TORQUE;

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
        motor.setNeutralMode(NeutralMode.Coast);
    }

    public Command resetEncoderForTesting(double angle) {
        return new InstantCommand(
                () -> motor.setSelectedSensorPosition(angleToTick(angle)));
    }

    public double getAngle() {
        return tickToAngle(motor.getSelectedSensorPosition());
    }

    public double getGroundAngle() {
        return interpolatedSetpoint;
    }

    public double getMass() {
        return mass;
    }

    public double getHigherMass() {
        return higherSegment == null ? mass
                : (mass + higherSegment.getHigherMass());
    }

    public Translation2d getRelativeEndpoint() {
        return new Translation2d(length, Rotation2d.fromDegrees(
                interpolatedSetpoint));
    }

    public Translation2d getRelativeCenterOfMass() {
        Rotation2d pivotPosition = Rotation2d.fromDegrees(interpolatedSetpoint);
        if (higherSegment == null) {
            return centerOfMass.rotateBy(pivotPosition);
        }

        double higherMass = higherSegment.getHigherMass();
        Translation2d higherCenterOfMass = higherSegment.getRelativeCenterOfMass()
                                                        .plus(getRelativeEndpoint());
        Translation2d scaledHigherCoM = higherCenterOfMass.times(higherMass);
        Translation2d scaledMyCoM = centerOfMass.rotateBy(pivotPosition).times(mass);
        Translation2d centerOfMassCalc = scaledHigherCoM.plus(scaledMyCoM)
                                                        .div(higherMass + mass);
        SmartDashboard.putNumber(name + " higher mass: ", higherMass);
        SmartDashboard.putString(name + " higher com: ",
                formatPolar(higherCenterOfMass));
        SmartDashboard.putString(name + " com calc: ",
                formatPolar(centerOfMassCalc));
        return centerOfMassCalc;
    }

    public double calculateKG() {
        return calculateKG(getRelativeCenterOfMass());
    }

    public double calculateKG(Translation2d totalCenterOfMass) {
        double torqueRequired = getHigherMass() * GRAVITY_ACCELERATION
                * totalCenterOfMass.getNorm();
        return torqueRequired / stallTorque * efficiency;
    }

    public double calculateFeedForward() {
        Translation2d totalCenterOfMass = getRelativeCenterOfMass();
        double kG = calculateKG(totalCenterOfMass);
        return kG * totalCenterOfMass.getAngle()
                                     .getCos();
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
        interpolatedSetpoint = getAngle();
        setpoint = angle;
        isSetPointCommanded = true;
    }

    private void setAngleOnMotor(double angle) {
        // double groundAngle = getGroundAngle();
        double FF = calculateFeedForward();
        // if (name.equalsIgnoreCase("base")) {
        //     motor.neutralOutput();
        // } else {
            motor.set(TalonFXControlMode.Position, angleToTick(angle),
                    DemandType.ArbitraryFeedForward, FF);
        // }
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
            interpolatedSetpoint = getAngle();
            setpoint = getAngle();
        }
        SmartDashboard.putNumber(name + " kG: ", calculateKG());
        SmartDashboard.putNumber(name + " angle: ", getAngle());
        SmartDashboard.putNumber(name + " setpoint: ", interpolatedSetpoint);
        SmartDashboard.putNumber(name + " ff", calculateFeedForward());
        SmartDashboard.putNumber(name + " current draw:", motor.getSupplyCurrent());
        Translation2d com = getRelativeCenterOfMass();
        SmartDashboard.putString(name + " Center of mass: ",
                String.format(formatPolar(com)));
        if (isSetPointCommanded) {
            double magnitude = Math.abs(interpolatedSetpoint - setpoint);
            if (magnitude < ANGULAR_VELOCITY_UNIT_TICKS) {
                interpolatedSetpoint = setpoint;
            } else if (interpolatedSetpoint > setpoint) {
                interpolatedSetpoint -= ANGULAR_VELOCITY_UNIT_TICKS;
            } else {
                interpolatedSetpoint += ANGULAR_VELOCITY_UNIT_TICKS;
            }
            setAngleOnMotor(interpolatedSetpoint);
        } else {
            motor.neutralOutput();
        }
    }

    private static String formatPolar(Translation2d t) {
        return String.format("ø=%4.1f, m=%4.2f", t.getAngle()
                                                  .getDegrees(),
                t.getNorm());
    }
}
