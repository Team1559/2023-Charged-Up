package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_WRIST;
import static frc.robot.Constants.Wiring.ARM_WRIST_CANCODER_ID;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.subsystems.arm.Arm.Position;

public class ArmWrist extends ArmSegment {

    public ArmWrist(Pigeon2 imu) {
        super("Wrist", ARM_MOTOR_ID_WRIST, ARM_WRIST_CANCODER_ID, imu, kP_WRIST, kI_WRIST, kD_WRIST,
                kIZ_WRIST, INV_ARM_WRIST_GEAR_RATIO, WRIST_SEGMENT_EFFICIENCY,
                MAXIMUM_VELOCITY_WRIST, ACCELERATION_WRIST, DECELERATION_WRIST, WRIST_SEGMENT_MASS,
                WRIST_SEGMENT_LENGTH, WRIST_SEGMENT_CENTER_OF_MASS, false, LOWER_ARM_WRIST_LIMIT,
                UPPER_ARM_WRIST_LIMIT, WRIST_CLOSED_LOOP_ERROR);
    }

    @Override
    protected double getTargetAngle(Position position) {
        return position.wrist;
    }
}
