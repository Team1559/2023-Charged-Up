package frc.robot.subsystems.arm;

import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.ARM_FALCON_ID_WRIST;
import static frc.robot.Constants.Wiring.WRIST_POTENTIOMETER_PORTNUM;
import static frc.robot.Constants.Arm.*;

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

public class ArmWrist extends SubsystemBase {
    private DTArmFeedforward    feedforward;
    private final DTTalonFX     wristMotor;
    private AnalogPotentiometer wristPotentiometer;
    private final double[]      wristPos = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    public ArmWrist() {
        wristMotor = new DTTalonFX(ARM_FALCON_ID_WRIST, kP_WRIST, kI_WRIST,
                kD_WRIST, 0);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(
                true, 40, 40, 0.5);
        wristMotor.configSupplyCurrentLimit(limit);

        feedforward = new DTArmFeedforward(kS_WRIST, kG_WRIST, kV_WRIST, 0);
        wristPotentiometer = new AnalogPotentiometer(
                WRIST_POTENTIOMETER_PORTNUM, ARM_WRIST_POTENTIOMETER_MULT,
                ARM_WRIST_POTENTIOMETER_ADD);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual degree offset right now

        this.addChild("Motor", wristMotor);
        this.addChild("Feedforward", feedforward);
        this.addChild("Potentiometer", wristPotentiometer);
    }

    public double getAngle() {
        return wristPotentiometer.get();
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * ARM_WRIST_GEAR_RATIO;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public void setAngle(double angle) {
        double FF = feedforward.calculate(angle, 0, 0);
        wristMotor.set(TalonFXControlMode.Position, angleToTick(angle),
                DemandType.ArbitraryFeedForward, FF);
    }

    public Command setWristAngleCommandPos(int angleIndex) {
        double angle = wristPos[angleIndex];
        return Commands.sequence(
                new InstantCommand(() -> setAngle(angle), this),
                new WaitCommand(
                        Math.abs(angle - getAngle()) / TELEOP_ANGLE_VELOCITY));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("U/D wrist potentiometer reading ",
                getAngle());
    }
}