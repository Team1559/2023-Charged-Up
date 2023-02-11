package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_WRIST;
import static frc.robot.Constants.Wiring.ARM_WRIST_CANCODER_ID;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Arm.*;

public class ArmWrist extends SubsystemBase {
    private final CANCoder      canCoder;
    private final TalonFX       wristMotor;
    private ArmFeedforward      feedforward;
    private final double[]      wristPos = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private int degrees;

    public ArmWrist() {
        canCoder = new CANCoder(ARM_WRIST_CANCODER_ID);
        wristMotor = new TalonFX(ARM_MOTOR_ID_WRIST);
        wristMotor.configFactoryDefault();
        wristMotor.enableVoltageCompensation(true);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.5);
        wristMotor.configSupplyCurrentLimit(limit);
        wristMotor.config_kP(0, kP_ELBOW);
        wristMotor.config_kI(0, kI_ELBOW);
        wristMotor.config_kD(0, kD_ELBOW);
        feedforward = new ArmFeedforward(kS_ELBOW, kG_ELBOW, kV_ELBOW, kA_ELBOW);
    }
    private void configCancoder() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = 0;
        config.sensorDirection = false;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoder.configAllSettings(config);
        canCoder.setPosition(
                canCoder.getAbsolutePosition() - BASE_CC_OFFSET);
    }
    public double getAngle() {
        return tickToAngle(wristMotor.getSelectedSensorPosition());
    }

    public static double angleToTick(double angle) {
        double revolutionsOfArm = angle / 360.0;
        double motorRevolutions = revolutionsOfArm * INV_GEAR_RATIO_BASE;
        double angleTicks = motorRevolutions * FALCON_TICKS_PER_REV;
        return angleTicks;
    }
    public static double tickToAngle(double ticks){
        double motorRevolutions = ticks / FALCON_TICKS_PER_REV;
        double revolutionsOfArm = motorRevolutions/INV_GEAR_RATIO_BASE;
        double angle = revolutionsOfArm * 360;
        return angle;
    }

    public void setAngle(double angle) {
        double FF = feedforward.calculate(angle, 0, 0);
        wristMotor.set(TalonFXControlMode.Position, angleToTick(angle),
                DemandType.ArbitraryFeedForward, FF);
    }

    public int degreeSetUp(){
        degrees += 10;
        if (degrees > 90){
            degrees = 90;
        }
        return degrees;
    }
    public int degreeSetDown(){
        degrees -= 10;
        if (degrees < 0){
            degrees = 0;
        }
        return degrees;
    }
    public Command degreesUp(){
        return Commands.sequence(
            new InstantCommand(() -> degreeSetUp(), this),
            new InstantCommand(() -> setAngle(degrees), this)
        );
    }
    public Command degreesDown(){
        return Commands.sequence(
            new InstantCommand(() -> degreeSetDown(), this),
            new InstantCommand(() -> setAngle(degrees), this)
        );
    }

    public Command setWristAngleCommandPos(int angleIndex) {
        double angle = wristPos[angleIndex];
        return new FunctionalCommand(() -> {setAngle(angle);}, () -> {}, (a) -> {}, () -> {return isArmWristAtPosition(angle);}, this);
    }
    public boolean isArmWristAtPosition(double angle){
        double angleError = Math.abs(angle - getAngle());
        if (angleError < 0.05){
            return true;
        }
        return false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist CANCoder reading (in deg):",
                getAngle());
    }
}
