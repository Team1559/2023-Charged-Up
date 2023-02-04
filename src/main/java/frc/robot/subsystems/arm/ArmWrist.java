package frc.robot.subsystems.arm;

import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Arm.ARM_WRIST_GEAR_RATIO;
import static frc.robot.Constants.Wiring.ARM_FALCON_ID_WRIST;
import static frc.robot.Constants.Wiring.WRIST_POTENTIOMETER_PORTNUM;
import static frc.robot.Constants.Arm.TELEOP_ANGLE_VELOCITY;
import static frc.robot.Constants.Arm.ARM_WRIST_POTENTIOMETER_MULT;
import static frc.robot.Constants.Arm.ARM_WRIST_POTENTIOMETER_ADD;
import static frc.robot.Constants.Arm.kP_WRIST;
import static frc.robot.Constants.Arm.kI_WRIST;
import static frc.robot.Constants.Arm.kD_WRIST;
import static frc.robot.Constants.Arm.kS_WRIST;
import static frc.robot.Constants.Arm.kG_WRIST;
import static frc.robot.Constants.Arm.kV_WRIST;

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

public class ArmWrist extends SubsystemBase {
    private ArmFeedforward      feedforward;
    private final TalonFX       wristMotor;
    private AnalogPotentiometer wristPotentiometer;

    private final double[] wristPos = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    public ArmWrist() {
        wristMotor = new TalonFX(ARM_FALCON_ID_WRIST);

        wristMotor.configFactoryDefault();
        wristMotor.enableVoltageCompensation(true);
        wristMotor.config_kP(0, kP_WRIST);
        wristMotor.config_kI(0, kI_WRIST);
        wristMotor.config_kD(0, kD_WRIST);
        feedforward = new ArmFeedforward(kS_WRIST, kG_WRIST, kV_WRIST);
        wristPotentiometer = new AnalogPotentiometer(
                WRIST_POTENTIOMETER_PORTNUM, 90, 0);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual degree offset right now
    }

    public double getAngle() {
        return ARM_WRIST_POTENTIOMETER_ADD
                + wristPotentiometer.get() * ARM_WRIST_POTENTIOMETER_MULT;
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * ARM_WRIST_GEAR_RATIO;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public void setAngle(double angle) {
        if (angle / 180.0 > 2) {

        } else {
            wristMotor.set(TalonFXControlMode.Position, angleToTick(angle));
        }
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