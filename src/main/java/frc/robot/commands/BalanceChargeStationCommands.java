package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swerve.SwerveDrive;

public class BalanceChargeStationCommands {
    private BalanceChargeStationCommands() {}

    public static Command autoBalanceCommand(SwerveDrive swerve) {
        return driveUntilOnTippedStation(swerve).andThen(driveUntilTippingDown(swerve, 0.5))
                                                .andThen(holdPositionUntilStable(swerve))
                                                .andThen(balanceWithPID(swerve))
                                                .andThen(new SwerveHoldPositionCommand(swerve));
    }

    public static Command driveOverChargeStation(SwerveDrive swerve) {
        return driveUntilOnTippedStation(swerve).andThen(driveUntilTippingDown(swerve, 1.5))
                                                .andThen(driveUntilLevel(swerve));

    }

    private static Command driveUntilOnTippedStation(SwerveDrive swerve) {
        return new SwerveDriveForwardCommand(swerve, 1, new BooleanSupplier() {
            private double maxPitch;

            @Override
            public boolean getAsBoolean() {
                double pitch = swerve.getGyroPitchDegrees();
                if (pitch > maxPitch) {
                    maxPitch = pitch;
                }
                return maxPitch >= 15 && pitch <= 15;
            }
        }).withTimeout(5);
    }

    private static Command driveUntilTippingDown(SwerveDrive swerve, double velocity) {
        return new SwerveDriveForwardCommand(swerve, velocity, new BooleanSupplier() {
            private int triggerCycleCount;

            @Override
            public boolean getAsBoolean() {
                if (swerve.getGyroPitchDegrees() <= 10) {
                    triggerCycleCount++;
                } else {
                    triggerCycleCount = 0;
                }
                return triggerCycleCount >= 5; // 0.1 seconds at target pitch
            }
        }).withTimeout(5);
    }

    private static Command holdPositionUntilStable(SwerveDrive swerve) {
        return new SwerveHoldPositionCommand(swerve).withTimeout(2);
    }

    private static Command balanceWithPID(SwerveDrive swerve) {
        // 1 consecutive second
        return new SwerveDrivePidBalanceCommand(swerve, 0.03, 0, 0, 50).withTimeout(10);
    }

    private static Command driveUntilLevel(SwerveDrive swerve) {
        return new SwerveDriveForwardCommand(swerve, 2, new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return Math.abs(swerve.getGyroPitchDegrees()) < 2;
            }
        }).withTimeout(2);
    }
}
