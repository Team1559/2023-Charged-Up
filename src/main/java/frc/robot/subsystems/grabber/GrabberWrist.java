package frc.robot.subsystems.grabber;

import static frc.robot.Constants.Grabber.MAX_ANGULAR_VELOCITY;
import static frc.robot.Constants.Grabber.SERVO_RANGE;
import static frc.robot.Constants.Grabber.ZERO_ANGLE;
import static frc.robot.Constants.Wiring.WRIST_MOTOR_PORT;
import static frc.robot.Constants.Wiring.*;


//import edu.wpi.first.wpilibj.Servo;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GrabberWrist extends SubsystemBase {
    //private Servo wristServo;
    private WPI_TalonSRX wristMotor;
    private CANCoder canCoder;
    private PIDController pidController;
    public GrabberWrist() {
        //wristServo = new Servo(WRIST_SERVO_PORT);
        wristMotor = new WPI_TalonSRX(WRIST_MOTOR_PORT);
        canCoder = new  CANCoder(WRIST_CANCODER_ID);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        pidController = new PIDController(0.1, 0.0, 0.0);
    }
    

    public void setAngle(double grabberAngle) {// Verify selected servo is 0-180
        double currentPosition = canCoder.getAbsolutePosition(); // find current position
        double pidValue = pidController.calculate(grabberAngle, currentPosition);
        wristMotor.set(pidValue);
    
        /** pass current position to PID controller and get value from PID representing amount of motor power
        *set motor to that value
        *kaboom?
        /* */
    }

    public double getAngle() {
        return canCoder.getAbsolutePosition();
        //returns from -180 to +180 ^
    }

    public Command setWristAngleCommand(double angle) {
        return Commands.sequence(
                new InstantCommand(() -> setAngle(angle), this),
                new WaitCommand(
                        Math.abs(angle - getAngle()) / MAX_ANGULAR_VELOCITY));
    }

    public void resetWrist() {
        //wristServo.setAngle(ZERO_ANGLE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Grabber Angle",
                wristMotor.get() * SERVO_RANGE - ZERO_ANGLE);
        SmartDashboard.putNumber("Grabber Servo Angle", wristMotor.get());
    }
    // private double calculateServoAngle(double targetAngle){
    // return targetAngle+ZERO_ANGLE;
    // }
}