package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

public class ArmBase extends SubsystemBase{
    private final TalonFX baseMotor;
    private final AnalogPotentiometer basePotentiometer;
    public ArmBase (){
        baseMotor = new TalonFX(ARM_MOTOR_ID_BASE);
        
        baseMotor.configFactoryDefault();
        baseMotor.config_kP(0, kP_BASE);
        baseMotor.config_kI(0, kI_BASE);
        baseMotor.config_kD(0, kD_BASE);
        baseMotor.config_kF(0, kF_BASE);

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
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Base potentiometer reading (in deg)", getAngle());
    }
}
