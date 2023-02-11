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

import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_BASE;
import static frc.robot.Constants.Wiring.BASE_CANCODER_ID;
import static frc.robot.Constants.Arm.INV_GEAR_RATIO_BASE;
import static frc.robot.Constants.FALCON_TICKS_PER_REV;
import static frc.robot.Constants.Arm.*;

public class ArmBase extends SubsystemBase {

    private final TalonFX             baseMotor;
    private final CANCoder canCoder;
    private final ArmFeedforward      feedforward;
    private final double[]            basePos = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
    private int degrees;
    
    public ArmBase() {
        canCoder = new CANCoder(BASE_CANCODER_ID);
        baseMotor = new TalonFX(ARM_MOTOR_ID_BASE);
        baseMotor.configFactoryDefault();
        baseMotor.enableVoltageCompensation(true);
        SupplyCurrentLimitConfiguration limit = new SupplyCurrentLimitConfiguration(true, 40, 60, 0.5);
        baseMotor.configSupplyCurrentLimit(limit);
        baseMotor.config_kP(0, kP_BASE);
        baseMotor.config_kI(0, kI_BASE);
        baseMotor.config_kD(0, kD_BASE);
        baseMotor.config_IntegralZone(0, angleToTick(kIZ_BASE));
        baseMotor.setSelectedSensorPosition(angleToTick(90));
        // baseMotor.setInverted(true);
        feedforward = new ArmFeedforward(kS_BASE, kG_BASE, kV_BASE, kA_BASE);
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
    public void resetEncoderForTesting(double angle) {
        baseMotor.setSelectedSensorPosition(angleToTick(angle));
    }
    public double getAngle() {
        return tickToAngle(baseMotor.getSelectedSensorPosition());
    }

    public static double angleToTick(double angle) {
        // Approx 524 ticks per degree
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
        double FF = feedforward.calculate(angle, 0, 0) / 12.0;
        SmartDashboard.putNumber("base feedforward", FF);
        baseMotor.set(TalonFXControlMode.Position, angleToTick(angle),
                DemandType.ArbitraryFeedForward, FF);
    }

    public Command setBaseAngleCommandPos(int angleIndex) {
        double angle = basePos[angleIndex];
        return new FunctionalCommand(() -> {setAngle(angle);}, () -> {}, (a) -> {}, () -> {return IsArmBaseAtPosition(angle);}, this);
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
    public boolean IsArmBaseAtPosition(double angle){
        double angleError = Math.abs(angle - getAngle());
        if (angleError < MAXIMUM_ANGLE_ERROR){
            return true;
        }
        return false;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Base angle: ", getAngle());
        SmartDashboard.putNumber("Base error: ", baseMotor.getClosedLoopError());
        SmartDashboard.putNumber("Base current: ", baseMotor.getStatorCurrent());
        SmartDashboard.putNumber("Base temperature: ", baseMotor.getTemperature());
    }
}
