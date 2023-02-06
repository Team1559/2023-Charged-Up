package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.DTTalonFX;

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.ELBOW_POTENTIOMETER_PORTNUM;
import static frc.robot.Constants.Arm.*;

public class ArmElbow extends SubsystemBase {

    private final DTTalonFX     elbowMotor;
    private AnalogPotentiometer elbowPotentiometer;
    private DTArmFeedforward    feedforward;
    private final double[]      elbowPos = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    public ArmElbow() {
        elbowMotor = new DTTalonFX(ARM_MOTOR_ID_ELBOW, kP_ELBOW, kI_ELBOW,
                kD_ELBOW, 0);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(
                true, 40, 60, 0.5);
        elbowMotor.configSupplyCurrentLimit(limit);
        feedforward = new DTArmFeedforward(kS_ELBOW, kG_ELBOW, kV_ELBOW,
                kA_ELBOW);

        elbowPotentiometer = new AnalogPotentiometer(
                ELBOW_POTENTIOMETER_PORTNUM, ARM_ELBOW_POTENTIOMETER_MULT,
                ARM_ELBOW_POTENTIOMETER_ADD);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual degree offset right now

        this.addChild("Motor", elbowMotor);
        this.addChild("Feedforward", feedforward);
        this.addChild("Potentiometer", elbowPotentiometer);
    }

    public double getAngle() {
        return elbowPotentiometer.get();
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

    public Command setElbowAngleCommandPos(int angleIndex) {
        double angle = elbowPos[angleIndex];
        return Commands.sequence(
                new InstantCommand(() -> setAngle(angle), this),
                new WaitCommand(
                        Math.abs(angle - getAngle()) / TELEOP_ANGLE_VELOCITY));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elbow potentiometer reading (in deg) ",
                getAngle());
    }
}
