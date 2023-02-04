package frc.robot.subsystems.arm;

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

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.ELBOW_POTENTIOMETER_PORTNUM;
import static frc.robot.Constants.Arm.TELEOP_ANGLE_VELOCITY;
import static frc.robot.Constants.Arm.ARM_ELBOW_POTENTIOMETER_MULT;
import static frc.robot.Constants.Arm.ARM_ELBOW_POTENTIOMETER_ADD;
import static frc.robot.Constants.Arm.kP_ELBOW;
import static frc.robot.Constants.Arm.kI_ELBOW;
import static frc.robot.Constants.Arm.kD_ELBOW;
import static frc.robot.Constants.Arm.kG_ELBOW;
import static frc.robot.Constants.Arm.kV_ELBOW;
import static frc.robot.Constants.Arm.kS_ELBOW;

public class ArmElbow extends SubsystemBase {

    private final TalonFX       elbowMotor;
    private AnalogPotentiometer elbowPotentiometer;
    private ArmFeedforward      feedforward;

    private final double[] elbowPos = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    public ArmElbow() {
        elbowMotor = new TalonFX(ARM_MOTOR_ID_ELBOW);

        elbowMotor.configFactoryDefault();
        elbowMotor.enableVoltageCompensation(true);
        elbowMotor.config_kP(0, kP_ELBOW);
        elbowMotor.config_kI(0, kI_ELBOW);
        elbowMotor.config_kD(0, kD_ELBOW);
        feedforward = new ArmFeedforward(kS_ELBOW, kG_ELBOW, kV_ELBOW);

        elbowPotentiometer = new AnalogPotentiometer(
                ELBOW_POTENTIOMETER_PORTNUM, 180, 0);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual degree offset right now
    }

    public double getAngle() {
        return ARM_ELBOW_POTENTIOMETER_ADD
                + elbowPotentiometer.get() * ARM_ELBOW_POTENTIOMETER_MULT;
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public void setAngle(double angle) {
        if (angle / 180.0 > elbowPotentiometer.get()) {

        } else {
            elbowMotor.set(TalonFXControlMode.Position, angleToTick(angle));
        }
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
