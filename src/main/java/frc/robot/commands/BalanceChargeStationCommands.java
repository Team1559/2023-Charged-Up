package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.SwerveDrive;

public class BalanceChargeStationCommands {
    private BalanceChargeStationCommands() {}

    public static Command autoBalanceCommand(SwerveDrive swerve) {
        return driveUntilOnTippedStation(swerve).andThen(driveUntilTippingDown(swerve))
                                                .andThen(holdPositionUntilStable(swerve))
                                                .andThen(balanceWithPID(swerve))
                                                .andThen(new SwerveHoldPositionCommand(swerve));
    }

    private static Command driveUntilOnTippedStation(SwerveDrive swerve) {
        return new SwerveDriveForwardCommand(swerve, 0.5, new BooleanSupplier() {
            private double maxPitch;

            @Override
            public boolean getAsBoolean() {
                double pitch = swerve.getGyroPitchDegrees();
                if (pitch > maxPitch) {
                    maxPitch = pitch;
                }
                return maxPitch >= 15 && pitch <= 9;
            }
        }).withTimeout(5);
    }

    private static Command driveUntilTippingDown(SwerveDrive swerve) {
        return new SwerveDriveForwardCommand(swerve, 0, new BooleanSupplier() {
            private int triggerCycleCount;

            @Override
            public boolean getAsBoolean() {
                if (swerve.getGyroPitchVelocity() < -5) {
                    triggerCycleCount++;
                } else {
                    triggerCycleCount = 0;
                }
                return triggerCycleCount >= 10; // 0.2 consecutive seconds of
                                                // movement
            }
        }).withTimeout(5);
    }

    private static Command holdPositionUntilStable(SwerveDrive swerve) {
        return new SwerveHoldPositionCommand(swerve).until(new BooleanSupplier() {
            private int triggerCycleCount;

            @Override
            public boolean getAsBoolean() {
                if (swerve.getGyroPitchVelocity() > -2) {
                    triggerCycleCount++;
                } else {
                    triggerCycleCount = 0;
                }
                return triggerCycleCount >= 10; // 0.2 consecutive seconds of
                                                // stability
            }
        })
                                                    .withTimeout(2);
    }

    private static Command balanceWithPID(SwerveDrive swerve) {
        // 1 consecutive second
        return new SwerveDrivePidBalanceCommand(swerve, 0, 0, 0, 50).withTimeout(10);
    }
}
