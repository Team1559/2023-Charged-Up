package frc.robot.subsystems.arm;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Wiring.ARM_MOTOR_ID_ELBOW;
import static frc.robot.Constants.Wiring.ELBOW_CANCODER_ID;
import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmElbow extends ArmSegment {
    private static final double[] elbowPos = { 0, 0, 0, 0, 0, -60, -45, -30, -15, 0 };

    public ArmElbow(ArmSegment previousSegment) {
        super("Elbow", ARM_MOTOR_ID_ELBOW, ELBOW_CANCODER_ID, kP_ELBOW,
                kI_ELBOW, kD_ELBOW, kIZ_ELBOW, INV_ELBOW_GEAR_RATIO,
                new ArmFeedforward(kS_ELBOW, kG_ELBOW, kV_ELBOW, kA_ELBOW),
                elbowPos, previousSegment);
    }
}
