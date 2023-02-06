package frc.lib;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class DTTalonFX extends WPI_TalonFX {
    private double kp;
    private double ki;
    private double kd;
    private double kf;

    public DTTalonFX(int canID, double kp, double ki, double kd, double kf) {
        super(canID);

        this.configFactoryDefault();
        this.enableVoltageCompensation(true);
        this.setP(kp);
        this.setI(ki);
        this.setD(kd);
        this.setF(kf);
    }

    public double getP() {
        return kp;
    }

    public double getI() {
        return ki;
    }

    public double getD() {
        return kd;
    }

    public double getF() {
        return kf;
    }

    public void setP(double x) {
        kp = x;
        config_kP(0, kp);
    }

    public void setI(double x) {
        ki = x;
        config_kI(0, x);
    }

    public void setD(double x) {
        kd = x;
        config_kD(0, x);
    }

    public void setF(double x) {
        kf = x;
        config_kF(0, x);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("DTTalonFX");
        builder.addDoubleProperty("kP", this::getP, this::setP);
        builder.addDoubleProperty("kI", this::getI, this::setI);
        builder.addDoubleProperty("kD", this::getD, this::setD);
        builder.addDoubleProperty("kF", this::getF, this::setF);
        builder.addDoubleProperty("Position", this::getSelectedSensorPosition,
                null);
        builder.addDoubleProperty("Velocity", this::getSelectedSensorVelocity,
                null);
    }

}
