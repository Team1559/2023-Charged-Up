package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.Wiring.ELBOW_CANCODER_ID;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.subsystems.arm.Arm.Position;

public class ArmElbow extends ArmSegment {

    public ArmElbow(Pigeon2 imu) {
        super("Elbow", ARM_MOTOR_ID_ELBOW, ELBOW_CANCODER_ID, imu, kP_ELBOW, kI_ELBOW, kD_ELBOW,
                kIZ_ELBOW, INV_ELBOW_GEAR_RATIO, ELBOW_SEGMENT_EFFICIENCY, MAXIMUM_VELOCITY_ELBOW,
                ACCELERATION_ELBOW, DECELERATION_ELBOW, ELBOW_SEGMENT_MASS, ELBOW_SEGMENT_LENGTH,
                ELBOW_SEGMENT_CENTER_OF_MASS, true, LOWER_ELBOW_LIMIT, UPPER_ELBOW_LIMIT,
                ELBOW_CLOSED_LOOP_ERROR, ELBOW_CANCODER_OFFSET);
    }

    @Override
    protected double getTargetAngle(Position position) {
        return position.elbow;
    }
}
