package frc.robot.subsystems.arm;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class DTArmFeedforward implements Sendable, AutoCloseable {
    public double ks;
    public double kg;
    public double kv;
    public double ka;

    public DTArmFeedforward(double ks, double kg, double kv, double ka) {
        this.ks = ks;
        this.kg = kg;
        this.kv = kv;
        this.ka = ka;
    }

    public double getG() {
        return kg;
    }

    public double getV() {
        return kv;
    }

    public double getA() {
        return ka;
    }

    public void setG(double x) {
        kg = x;
    }

    public void setV(double x) {
        kv = x;
    }

    public void setA(double x) {
        ka = x;
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ArmFeedforward");
        builder.addDoubleProperty("kG", this::getG, this::setG);
        builder.addDoubleProperty("kV", this::getV, this::setV);
        builder.addDoubleProperty("kA", this::getA, this::setA);
    }

    // WPILib methods from ArmFeedforward
    public double calculate(double positionRadians, double velocityRadPerSec,
            double accelRadPerSecSquared) {
        return ks * Math.signum(velocityRadPerSec)
                + kg * Math.cos(positionRadians) + kv * velocityRadPerSec
                + ka * accelRadPerSecSquared;
    }

    public double calculate(double positionRadians, double velocity) {
        return calculate(positionRadians, velocity, 0);
    }

    public double maxAchievableVelocity(double maxVoltage, double angle,
            double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka)
                / kv;
    }

    public double minAchievableVelocity(double maxVoltage, double angle,
            double acceleration) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + ks - Math.cos(angle) * kg - acceleration * ka)
                / kv;
    }

    public double maxAchievableAcceleration(double maxVoltage, double angle,
            double velocity) {
        return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg
                - velocity * kv) / ka;
    }

    public double minAchievableAcceleration(double maxVoltage, double angle,
            double velocity) {
        return maxAchievableAcceleration(-maxVoltage, angle, velocity);
    }
}
