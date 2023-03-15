package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.ACCELERATION_WRIST;
import static frc.robot.Constants.Arm.INV_ARM_WRIST_GEAR_RATIO;
import static frc.robot.Constants.Arm.LOWER_ARM_WRIST_LIMIT;
import static frc.robot.Constants.Arm.MAXIMUM_VELOCITY_WRIST;
import static frc.robot.Constants.Arm.UPPER_ARM_WRIST_LIMIT;
import static frc.robot.Constants.Arm.WRIST_CLOSED_LOOP_ERROR;
import static frc.robot.Constants.Arm.WRIST_SEGMENT_CENTER_OF_MASS;
import static frc.robot.Constants.Arm.WRIST_SEGMENT_EFFICIENCY;
import static frc.robot.Constants.Arm.WRIST_SEGMENT_LENGTH;
import static frc.robot.Constants.Arm.WRIST_SEGMENT_MASS;
import static frc.robot.Constants.Arm.kD_WRIST;
import static frc.robot.Constants.Arm.kIZ_WRIST;
import static frc.robot.Constants.Arm.kI_WRIST;
import static frc.robot.Constants.Arm.kP_WRIST;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_WRIST;
import static frc.robot.Constants.Wiring.ARM_WRIST_CANCODER_ID;

import frc.robot.subsystems.arm.Arm.Position;

public class ArmWrist extends ArmSegment {

    public ArmWrist() {
        super("Wrist", ARM_MOTOR_ID_WRIST, ARM_WRIST_CANCODER_ID, kP_WRIST, kI_WRIST, kD_WRIST,
                kIZ_WRIST, INV_ARM_WRIST_GEAR_RATIO, WRIST_SEGMENT_EFFICIENCY,
                MAXIMUM_VELOCITY_WRIST, ACCELERATION_WRIST, WRIST_SEGMENT_MASS,
                WRIST_SEGMENT_LENGTH, WRIST_SEGMENT_CENTER_OF_MASS, false, LOWER_ARM_WRIST_LIMIT,
                UPPER_ARM_WRIST_LIMIT, WRIST_CLOSED_LOOP_ERROR);
    }

    @Override
    protected double getTargetAngle(Position position) {
        return position.wrist;
    }
}
