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

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.BASE_POTENTIOMETER_PORTNUM;
import static frc.robot.Constants.Arm.ARM_BASE_POTENTIOMETER_MULT;
import static frc.robot.Constants.Arm.ARM_BASE_POTENTIOMETER_ADD;
import static frc.robot.Constants.Arm.kP_BASE;
import static frc.robot.Constants.Arm.kI_BASE;
import static frc.robot.Constants.Arm.kD_BASE;
import static frc.robot.Constants.Arm.kF_BASE;
import static frc.robot.Constants.Arm.kG_BASE;
import static frc.robot.Constants.Arm.kV_BASE;
import static frc.robot.Constants.Arm.kS_BASE;
import static frc.robot.Constants.Arm.TELEOP_ANGLE_VELOCITY;
public class ArmBase extends SubsystemBase{
    
    private final TalonFX baseMotor;
    private final AnalogPotentiometer basePotentiometer;

    private final double[] basePos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    public ArmBase (){
        baseMotor = new TalonFX(ARM_MOTOR_ID_BASE);
        
        baseMotor.configFactoryDefault();
        baseMotor.enableVoltageCompensation(true);
        baseMotor.config_kP(0, kP_BASE);
        baseMotor.config_kI(0, kI_BASE);
        baseMotor.config_kD(0, kD_BASE);
        baseMotor.config_kF(0, kF_BASE);
        ArmFeedforward feedforward = new ArmFeedforward(kS_BASE, kG_BASE, kV_BASE);
        
        basePotentiometer = new AnalogPotentiometer(BASE_POTENTIOMETER_PORTNUM, 180, 0); 
        //offset 0 is a placeholder, due to the fact we have no means of determining actual voltage right now
    }
    public double getAngle(){
         return ARM_BASE_POTENTIOMETER_ADD + basePotentiometer.get() * ARM_BASE_POTENTIOMETER_MULT;
    }
    public static double angleToTick(double angle){
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }
    public void setAngle(double angle){ //This probably won't work like I think it will.
        if (angle > 180 - (basePotentiometer.get() * 180)){ //Assuming angle will be commanded in degrees 0 -> 360, potentiometer on 0 -> 1 basis
        }
        else{
            baseMotor.set(TalonFXControlMode.Position, angleToTick(angle));
        }
    }
    public Command setBaseAngleCommandPos1(double angle){
        angle = basePos[0];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[0]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos2(double angle){
        angle = basePos[1];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[1]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos3(double angle){
        angle = basePos[2];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[2]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos4(double angle){
        angle = basePos[3];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[3]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos5(double angle){
        angle = basePos[4];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[4]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos6(double angle){
        angle = basePos[5];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[5]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos7(double angle){
        angle = basePos[6];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[6]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos8(double angle){
        angle = basePos[7];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[7]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos9(double angle){
        angle = basePos[8];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[8]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    public Command setBaseAngleCommandPos10(double angle){
        angle = basePos[9];
        return Commands.sequence(
            new InstantCommand(() -> setAngle(basePos[9]), this),
            new WaitCommand(Math.abs(angle - getAngle()) /  TELEOP_ANGLE_VELOCITY)); 
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Base potentiometer reading (in deg)", getAngle());
    }
}
