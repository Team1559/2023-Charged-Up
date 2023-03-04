package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Arm {
    public enum positionNames {
        UPPER_CONE,
        MIDDLE_CONE,
        LOWER_CONE,

        LOWER_CUBE,
        MIDDLE_CUBE,
        UPPER_CUBE,

        WAYPOINT,

        TRAVEL,
        GAME_START,
        PRE_PICKUP,
        PICKUP
    }

    private double[][] positions = {
            /** cone 3 */
            { 51, -26, -8 },
            /** cone 2 */
            { 76, -46, -66 },
            /** cone 1 */
            { 87, -118, -34 },
            /** cube 1 */
            { 83, -119, -38 },
            /** cube 2 */
            { 70, -59, -71 },
            /** cube 3 */
            { 49, -17, -62 },
            /** waypoint */
            { 94, -58, -89 },
            /** travel */
            { 94, -118, -87 },
            /** game start */
            { 94, -33, -68 },
            /** pre-pickup */
            { 94, -117, -90 },
            /** pickup */
            { 94, -140, -45 } };
    private ArmBase    base;
    private ArmElbow   elbow;
    private ArmWrist   wrist;

    public Arm(ArmBase base, ArmElbow elbow, ArmWrist wrist) {
        this.base = base;
        this.elbow = elbow;
        this.wrist = wrist;
    }

    public Command moveArmToPosition(int index) {
        double[] angles = positions[index];
        return new ParallelCommandGroup(base.setAngleCommandPos(angles[0]),
                elbow.setAngleCommandPos(angles[1]), wrist.setAngleCommandPos(angles[2]));
    }

    public SequentialCommandGroup moveToLocations(int... positions) {
        Command[] groupOfCommands = new Command[positions.length];
        for (int i = 0; i < groupOfCommands.length; i++) {
            groupOfCommands[i] = moveArmToPosition(positions[i]);
        }
        return new SequentialCommandGroup(groupOfCommands);
    }

}
