package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
import static frc.robot.Constants.Arm.kF_ELBOW;
import static frc.robot.Constants.Arm.kG_ELBOW;
import static frc.robot.Constants.Arm.kV_ELBOW;
import static frc.robot.Constants.Arm.kS_ELBOW;

public class ArmElbow extends SubsystemBase{
    
    private final TalonFX       elbowMotor;
    private AnalogPotentiometer elbowPotentiometer;

    private final double[] elbowPos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    public ArmElbow(){
        elbowMotor = new TalonFX(ARM_MOTOR_ID_ELBOW);
        
        elbowMotor.configFactoryDefault();
        elbowMotor.enableVoltageCompensation(true);
        elbowMotor.config_kP(0, kP_ELBOW);
        elbowMotor.config_kI(0, kI_ELBOW);
        elbowMotor.config_kD(0, kD_ELBOW);
        elbowMotor.config_kF(0, kF_ELBOW);
        ArmFeedforward feedforward = new ArmFeedforward(kS_ELBOW, kG_ELBOW, kV_ELBOW);

        elbowPotentiometer = new AnalogPotentiometer(ELBOW_POTENTIOMETER_PORTNUM, 180, 0);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual degree offset right now
    }

    public double getAngle() {
        return ARM_ELBOW_POTENTIOMETER_ADD + elbowPotentiometer.get() * ARM_ELBOW_POTENTIOMETER_MULT;
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

    public Command setElbowAngleCommandPos1(double angle){
        angle = elbowPos[0];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[0]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos2(double angle){
        angle = elbowPos[1];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[1]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos3(double angle){
        angle = elbowPos[2];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[2]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos4(double angle){
        angle = elbowPos[3];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[3]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos5(double angle){
        angle = elbowPos[4];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[4]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos6(double angle){
        angle = elbowPos[5];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[5]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos7(double angle){
        angle = elbowPos[6];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[6]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos8(double angle){
        angle = elbowPos[7];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[7]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos9(double angle){
        angle = elbowPos[8];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[8]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setElbowAngleCommandPos10(double angle){
        angle = elbowPos[9];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(elbowPos[9]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elbow potentiometer reading (in deg) ", getAngle());
    }
}
