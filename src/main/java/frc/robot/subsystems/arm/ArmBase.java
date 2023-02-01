package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.BASE_POTENTIOMETER_PORTNUM;

public class ArmBase{
    private final TalonFX baseMotor;
    private final AnalogPotentiometer basePotentiometer;
    public ArmBase (){
        baseMotor = new TalonFX(ARM_MOTOR_ID_BASE);
        baseMotor.configFactoryDefault();
        basePotentiometer = new AnalogPotentiometer(BASE_POTENTIOMETER_PORTNUM, 180, 0); 
        //offset 0 is a placeholder, due to the fact we have no means of determining actual voltage right now
    }
    public double GetAngle(){
        return 0;
    }
    public static double angleToTick(double angle){
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }
    public void moveToAngle(double angle){ //This probably won't work like I think it will.
        if (angle > 180 - (basePotentiometer.get() * 180)){ //Assuming angle will be commanded in degrees 0 -> 360, potentiometer on 0 -> 1 basis
        }
        else{
            baseMotor.set(TalonFXControlMode.Position, angleToTick(angle));
        }
    }
}
