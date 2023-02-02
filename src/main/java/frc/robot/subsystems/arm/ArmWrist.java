package frc.robot.subsystems.arm;

import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Arm.ARM_WRIST_GEAR_RATIO;
import static frc.robot.Constants.Wiring.ARM_FALCON_ID_WRIST;
import static frc.robot.Constants.Wiring.WRIST_POTENTIOMETER_PORTNUM;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;


public class ArmWrist extends SubsystemBase {
    private static final double TELEOP_ANGLE_VELOCITY = 0;
    private final TalonFX baseMotor;
    private AnalogPotentiometer wristPotentiometer;

    public ArmWrist(){
        baseMotor = new TalonFX(ARM_FALCON_ID_WRIST);
        baseMotor.configFactoryDefault();
        wristPotentiometer = new AnalogPotentiometer(WRIST_POTENTIOMETER_PORTNUM, 90, 0);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual degree offset right now
    }

    public double getWrist() {
        return wristPotentiometer.get();
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * ARM_WRIST_GEAR_RATIO; //Inverted, we don't know actual ratio yet, so
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;       //ARM_WRIST_GEAR_RATIO is set to 1.
        return angleTicks;
    }
    public void setAngle(double angle) {
        if (angle / 180.0 > wristPotentiometer.get()) {

        } else {
            baseMotor.set(TalonFXControlMode.Position, angleToTick(angle));
        }
    }
    public Command setWristAngleCommand(double angle){
        return Commands.sequence(
            new InstantCommand(() -> setAngle(angle), this),
            new WaitCommand(Math.abs(angle - getWrist()) /  Constants.Arm.TELEOP_ANGLE_VELOCITY)); 
    }

    public void setDefaultCommand(Command teleOpWristCommand) {}
}