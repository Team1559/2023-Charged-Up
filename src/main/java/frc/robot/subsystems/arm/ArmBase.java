package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.BASE_POTENTIOMETER_PORTNUM;
import static frc.robot.Constants.Arm.ARM_BASE_POTENTIOMETER_MULT;
import static frc.robot.Constants.Arm.ARM_BASE_POTENTIOMETER_ADD;
import static frc.robot.Constants.Arm.kP_BASE;
import static frc.robot.Constants.Arm.kI_BASE;
import static frc.robot.Constants.Arm.kD_BASE;
import static frc.robot.Constants.Arm.kG_BASE;
import static frc.robot.Constants.Arm.kV_BASE;
import static frc.robot.Constants.Arm.kS_BASE;
import static frc.robot.Constants.Arm.kA_BASE;
import static frc.robot.Constants.Arm.TELEOP_ANGLE_VELOCITY;

public class ArmBase extends SubsystemBase {

    private final TalonFX             baseMotor;
    private final AnalogPotentiometer basePotentiometer;
    private final ArmFeedforward      feedforward;
    private final double[]            basePos = { 90, 75, 60, 0, 0, 0, 0, 0, 0,
            0 };

    public ArmBase() {
        baseMotor = new TalonFX(ARM_MOTOR_ID_BASE);

        baseMotor.configFactoryDefault();
        baseMotor.enableVoltageCompensation(true);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(
                true, 40, 80, 0.5);
        baseMotor.configSupplyCurrentLimit(limit);
        baseMotor.config_kP(0, kP_BASE);
        baseMotor.config_kI(0, kI_BASE);
        baseMotor.config_kD(0, kD_BASE);
        feedforward = new ArmFeedforward(kS_BASE, kG_BASE, kV_BASE, kA_BASE);

        basePotentiometer = new AnalogPotentiometer(BASE_POTENTIOMETER_PORTNUM,
                ARM_BASE_POTENTIOMETER_MULT, ARM_BASE_POTENTIOMETER_ADD);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual voltage right now
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
