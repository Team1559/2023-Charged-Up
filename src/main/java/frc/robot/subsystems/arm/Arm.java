package frc.robot.subsystems.arm;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.NullCommand;

public class Arm {
    public enum Position {
        LOWER_CONE(87, -118, -34, false),
        MIDDLE_CONE(76, -46, -66, true),
        UPPER_CONE(51, -26, -8, true),

        LOWER_CUBE(83, -119, -38, false),
        MIDDLE_CUBE(70, -59, -71, true),
        UPPER_CUBE(49, -17, -62, true),

        WAYPOINT(94, -58, -89, false),

        TRAVEL(94, -118, -87, false),
        GAME_START(94, -33, -68, false),
        PRE_PICKUP(94, -117, -90, false),
        PICKUP(94, -140, -45, false);

        public final double base;
        public final double elbow;
        public final double wrist;

        /** INVALID for {@code WAYPOINT} */
        public final boolean isUpper;

        Position(double base, double elbow, double wrist, boolean isUpper) {
            this.base = base;
            this.elbow = elbow;
            this.wrist = wrist;

            this.isUpper = isUpper;
        }
    }

    private final ArmBase  base;
    private final ArmElbow elbow;
    private final ArmWrist wrist;

    private Position targetPosition;
    private Position lastPosition;

    public Arm(ArmBase base, ArmElbow elbow, ArmWrist wrist) {
        this.base = base;
        this.elbow = elbow;
        this.wrist = wrist;
    }

    public Command moveArmToPosition(Position position) {
        return new SetPositionCommand(position);
    }

    public Command moveToLocations(Position... positions) {
        Command[] groupOfCommands = new Command[positions.length];
        for (int i = 0; i < groupOfCommands.length; i++) {
            groupOfCommands[i] = moveArmToPosition(positions[i]);
        }
        return new SequentialCommandGroup(groupOfCommands);
    }

    public Command moveSequentially(Position position) {
        return new SequentialArmCommand(position);
    }

    private class SetPositionCommand extends CommandBase {
        private final Arm.Position         position;
        private final ParallelCommandGroup commandGroup;

        SetPositionCommand(Arm.Position position) {
            this.position = position;
            this.commandGroup = new ParallelCommandGroup(base.setAngleCommandPos(position),
                    elbow.setAngleCommandPos(position), wrist.setAngleCommandPos(position));
        }

        @Override
        public void initialize() {
            targetPosition = position;
            CommandScheduler.getInstance()
                            .schedule(commandGroup);
        }

        @Override
        public boolean isFinished() {
            return commandGroup.isFinished();
        }

        @Override
        public void end(boolean interrupted) {
            if (!interrupted) {
                lastPosition = targetPosition;
            }
        }
    }

    private class SequentialArmCommand extends CommandBase {
        private final Arm.Position destination;
        private Command            commandToExecute;

        public SequentialArmCommand(Arm.Position destPos) {
            if (destPos == Position.WAYPOINT) {
                throw new IllegalArgumentException("WAYPOINT is invalid for a sequence command");
            }
            destination = Objects.requireNonNull(destPos);
        }

        @Override
        public void initialize() {
            if (destination == targetPosition) {
                // Command will be canceled by this one; need to restart it
                commandToExecute = moveArmToPosition(destination);
                CommandScheduler.getInstance()
                                .schedule(commandToExecute);
                return;
            }

            // Logic to determine if arm is currently in the upper or lower range
            boolean isUpper;
            if (lastPosition == targetPosition || lastPosition == Arm.Position.WAYPOINT) {
                // Not moving or moving away from WAYPOINT
                isUpper = targetPosition.isUpper;
            } else {
                // Moving towards WAYPOINT or within a group (upper/lower)
                isUpper = lastPosition.isUpper;
            }

            if (isUpper != destination.isUpper) {
                commandToExecute = moveToLocations(Arm.Position.WAYPOINT, destination);
            } else {
                commandToExecute = moveArmToPosition(destination);
            }

            CommandScheduler.getInstance()
                            .schedule(commandToExecute);
        }

        @Override
        public boolean isFinished() {
            return commandToExecute.isFinished();
        }
    }
}
