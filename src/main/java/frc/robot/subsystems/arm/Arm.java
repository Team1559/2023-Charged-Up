package frc.robot.subsystems.arm;

import java.util.Arrays;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Arm {
    public enum Position {
        LOWER_CONE(87, -118, -34, false),
        MIDDLE_CONE(76, -46, -66, true),
        UPPER_CONE(55, -26, -20, true),

        LOWER_CUBE(83, -119, -38, false),
        MIDDLE_CUBE(70, -59, -71, true),
        UPPER_CUBE(49, -17, -62, true),

        WAYPOINT(94, -68, -52, false),

        TRAVEL(99.5, -122.5, -78, false),
        PRE_PICKUP(99.5, -122.5, -78, false),
        GAME_START(99.5, -149, -27, false),
        // PRE_PICKUP(99.5, -117, -80, false),
        PICKUP_CONE(99.5, -144, -39, false),
        PICKUP_CUBE(99.5, -140, -52, false);

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

    private Position lastPosition;
    private Position targetPosition;
    private Position destinationPosition;

    public Arm(ArmBase base, ArmElbow elbow, ArmWrist wrist) {
        this.base = base;
        this.elbow = elbow;
        this.wrist = wrist;

        this.lastPosition = Position.GAME_START;
        this.targetPosition = Position.GAME_START;
        this.destinationPosition = Position.GAME_START;
    }

    public boolean needWaypoint() {
        boolean isUpper;
        if (lastPosition == targetPosition || lastPosition == Arm.Position.WAYPOINT) {
            // Not moving or moving away from WAYPOINT
            isUpper = targetPosition.isUpper;
        } else {
            // Moving towards WAYPOINT or within a group (upper/lower)
            isUpper = lastPosition.isUpper;
        }

        System.out.println("lastPos: " + lastPosition);
        System.out.println("targetPos: " + targetPosition);
        System.out.println("destPos: " + destinationPosition);
        System.out.println("isUpper: " + isUpper);
        System.out.println("Waypoint: " + (isUpper != destinationPosition.isUpper));
        System.out.println();

        return isUpper != destinationPosition.isUpper;
    }

    public Command moveToPosition(Position position) {
        return base.setAngleCommandPos(position)
                   .alongWith(elbow.setAngleCommandPos(position))
                   .alongWith(wrist.setAngleCommandPos(position))
                   .beforeStarting(() -> {
                       System.out.println("beforeStarting " + position);
                       targetPosition = position;
                   })
                   .finallyDo(i -> {
                       if (!i) {
                           lastPosition = targetPosition;
                       }
                   });
    }

    public Command moveToLocations(Position... positions) {
        return new SequentialCommandGroup(Arrays.stream(positions)
                                                .map(this::moveToPosition)
                                                .toArray(Command[]::new));
    }

    public Command moveSequentially(Position position) {
        return new SelectCommand(
                Map.ofEntries(Map.entry(true, moveToLocations(Position.WAYPOINT, position)),
                        Map.entry(false, moveToPosition(position))),
                this::needWaypoint).beforeStarting(() -> destinationPosition = position);
    }

    public void armPanic() {
        base.armPanic();
        elbow.armPanic();
        wrist.armPanic();
    }

    public Command armPanicCommand() {
        return new InstantCommand(this::armPanic).andThen(new WaitCommand(1.0));
    }
}
