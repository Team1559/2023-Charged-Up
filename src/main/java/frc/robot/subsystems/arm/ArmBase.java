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

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.BASE_POTENTIOMETER_PORTNUM;
import static frc.robot.Constants.Arm.*;

public class ArmBase extends SubsystemBase {
    private final DTTalonFX           baseMotor;
    private final AnalogPotentiometer basePotentiometer;
    private final DTArmFeedforward    feedforward;
    private final double[]            basePos = { 90, 75, 60, 0, 0, 0, 0, 0, 0,
            0 };

    public ArmBase() {
        baseMotor = new DTTalonFX(ARM_MOTOR_ID_BASE, kP_BASE, kI_BASE, kD_BASE,
                0);

        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(
                true, 40, 80, 0.5);
        baseMotor.configSupplyCurrentLimit(limit);

        feedforward = new DTArmFeedforward(kS_BASE, kG_BASE, kV_BASE, kA_BASE);

        basePotentiometer = new AnalogPotentiometer(BASE_POTENTIOMETER_PORTNUM,
                ARM_BASE_POTENTIOMETER_MULT, ARM_BASE_POTENTIOMETER_ADD);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual voltage right now

        this.addChild("Motor", baseMotor);
        this.addChild("Feedforward", feedforward);
        this.addChild("Potentiometer", basePotentiometer);
    }

    public double getAngle() {
        return basePotentiometer.get();
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public void setAngle(double angle) {
        double FF = feedforward.calculate(angle, 0, 0) / 12.0;
        baseMotor.set(TalonFXControlMode.Position, angleToTick(angle),
                DemandType.ArbitraryFeedForward, FF);
    }

    public Command setBaseAngleCommandPos(int angleIndex) {
        double angle = basePos[angleIndex];
        return Commands.sequence(
                new InstantCommand(() -> setAngle(angle), this),
                new WaitCommand(
                        Math.abs(angle - getAngle()) / TELEOP_ANGLE_VELOCITY));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Base potentiometer reading (Analog): ",
                getAngle());
    }
}
