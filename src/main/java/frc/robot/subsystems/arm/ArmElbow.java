package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class ArmElbow {
    private final TalonFX baseMotor;
    public ArmElbow (){
        baseMotor = new TalonFX(Constants.Wiring.ARM_MOTOR_ID_ELBOW);
        baseMotor.configFactoryDefault();
    }
    public double GetAngle(){
        return 0;
    }
    public static double angleToTick(double angle){
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * Constants.Arm.INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * Constants.FALCON_TICKS_PER_REV;
        return angleTicks;
    }
    public void moveToAngle(double angle){
        baseMotor.set(TalonFXControlMode.Position, angleToTick(angle));
    }
}
