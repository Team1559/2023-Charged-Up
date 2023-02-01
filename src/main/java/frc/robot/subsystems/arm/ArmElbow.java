package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Wiring.ELBOW_POTENTIOMETER_PORTNUM;

public class ArmElbow {
    private final TalonFX       baseMotor;
    private AnalogPotentiometer elbowPotentiometer;

    public ArmElbow() {
        baseMotor = new TalonFX(ARM_MOTOR_ID_ELBOW);
        baseMotor.configFactoryDefault();
        elbowPotentiometer = new AnalogPotentiometer(
                ELBOW_POTENTIOMETER_PORTNUM, 180, 0);
        // offset 0 is a placeholder, due to the fact we have no means of
        // determining actual degree offset right now
    }

    public double GetAngle() {
        return 0; // Temporary value until a potentiometer reading can be
                  // obtained.
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }

    public void moveToAngle(double angle) {
        if (angle / 180.0 > elbowPotentiometer.get()) {

        } else {
            baseMotor.set(TalonFXControlMode.Position, angleToTick(angle));
        }
    }
}
