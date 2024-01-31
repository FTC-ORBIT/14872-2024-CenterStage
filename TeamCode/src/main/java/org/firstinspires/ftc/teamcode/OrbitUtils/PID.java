package org.firstinspires.ftc.teamcode.OrbitUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    private static final ElapsedTime timer = new ElapsedTime();
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;
    public double iZone = 0;

    public double wanted = 0;

    private double integral = 0;

    private double prevError = 0;
    private double prevTime = 0;

    public PID(final double kP, final double kI, final double kD, final double kF, final double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
    }

    public void setWanted(final double wanted) {
        this.wanted = wanted;
    }

    public double update(final double current, final Telemetry telemetry) {
        telemetry.addData("wanted", wanted);
        telemetry.addData("current", current);
        final double currentError = wanted - current;
        final double currentTime = timer.milliseconds();
        final double deltaTime = currentTime - prevTime;

        if (Math.abs(currentError) < iZone) {
            if (Math.signum(currentError) != Math.signum(prevError)) {
                integral = 0;
            } else {
                integral += currentError * deltaTime;
            }
        }

        final double derivative = deltaTime == 0 ? 0 : (currentError - prevError) / deltaTime;

        prevError = currentError;
        prevTime = currentTime;

        return kP * currentError + kI * integral + kD * derivative + kF * wanted;
    }
}
