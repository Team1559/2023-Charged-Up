package frc.robot.subsystems.grabber;

import static frc.robot.Constants.Grabber.SERVO_RANGE;
import static frc.robot.Constants.Grabber.ZERO_ANGLE;
import static frc.robot.Constants.Wiring.WRIST_SERVO_PORT;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberWrist extends SubsystemBase {
    private Servo wristServo;

    // remove
    // private WPI_TalonSRX wristMotor;
    // private CANCoder canCoder;
    // private PIDController pidController;
    // remove
    public GrabberWrist() {
        wristServo = new Servo(WRIST_SERVO_PORT);
        // remove
        // wristMotor = new WPI_TalonSRX(WRIST_MOTOR_PORT);
        // canCoder = new CANCoder(WRIST_CANCODER_ID);
        // canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        // pidController = new PIDController(0.1, 0.0, 0.0);
        // remove
    }

    public void setAngle(double grabberAngle) {// Verify selected servo is 0-180

        wristServo.setAngle(grabberAngle + ZERO_ANGLE);
        // remove
        // * if not use
        // * wristServo.set()
        // * instead (0.0 to 1.0)
        // */
        // double currentPosition = canCoder.getAbsolutePosition(); // find
        // current position
        // double pidValue = pidController.calculate(grabberAngle,
        // currentPosition);
        // wristMotor.set(pidValue);
        // remove
    }

    public double getAngle() {
        return wristServo.getAngle() - ZERO_ANGLE;
        // remove
        // return canCoder.getAbsolutePosition();
        // returns from -180 to +180 ^
        // remove
    }

    public Command setWristAngleCommand(double angle) {
        return Commands.sequence(new InstantCommand(() -> setAngle(angle), this));
        // remove
        // new WaitCommand(
        // Math.abs(angle - getAngle()) / MAX_ANGULAR_VELOCITY));
        // remove
    }

    public void resetWrist() {
        wristServo.setAngle(ZERO_ANGLE);
    }

    @Override
    public void periodic() {
        // remove
        // SmartDashboard.putNumber("Grabber Angle",wristMotor.get() *
        // SERVO_RANGE - ZERO_ANGLE);
        // remove
        SmartDashboard.putNumber("Grabber Angle", wristServo.get() * SERVO_RANGE - ZERO_ANGLE);
        SmartDashboard.putNumber("Grabber Servo Angle", wristServo.getAngle()); // remove
    }

    // remove
    // private double calculateServoAngle(double targetAngle){
    // return targetAngle+ZERO_ANGLE;
    // }
    // remove
}
