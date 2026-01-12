package org.frogforce503.lib.motorcontrol;

import edu.wpi.first.wpilibj.Timer;

/**
 * Create a very basic motor simulation which can take in several setpoints and take some time to reach them.
 * This is physically not very accurate, but can be useful for testing.
 */
public class SimpleMotorSim {
    private final double timeForOne; // Number of seconds to complete a full revolution
    private final double timeConstantFactor; // Motor is modeled as logarithmic growth (like a charging capacitor)
    private final double CPR;

    private double currentPosition = 0;
    private double currentVelocity = 0;

    private Setpoint lastCommand;
    private double lastUpdateTime;
    private double lastPosition = 0;

    public SimpleMotorSim(double timeConstantFactor, double CPR) {
        this.timeConstantFactor = timeConstantFactor;
        this.timeForOne = Math.sqrt(timeConstantFactor); // https://www.electronics-tutorials.ws/rc/rc_1.html
        this.CPR = CPR;

        this.lastCommand = new Setpoint(Timer.getFPGATimestamp(), MotorControlMode.DutyCycle, 0, 0);
        this.lastUpdateTime = Timer.getFPGATimestamp();
    }

    public void set(MotorControlMode controlMode, double value) {
        if (controlMode == MotorControlMode.Position) {
            value /= CPR;
        } else if (controlMode == MotorControlMode.Velocity) {
            value *= 60;
        }

        lastCommand = new Setpoint(Timer.getFPGATimestamp(), controlMode, value, this.currentPosition);
    }

    public void update() {
        // Updates speed and position
        double time = Timer.getFPGATimestamp();

        switch(lastCommand.controlMode) {
            case DutyCycle: {
                this.currentVelocity = (1 / this.timeForOne) * this.lastCommand.demand; // basically, 1.0 power is max velocity
                this.currentPosition += (time - this.lastUpdateTime) * this.currentVelocity;
                break;
            }
            case Voltage: {
                this.currentVelocity = (1 / this.timeForOne) * this.lastCommand.demand / 12.0; // basically, 12 Volts is max velocity
                this.currentPosition += (time - this.lastUpdateTime) * this.currentVelocity;
                break;
            }
            case Current: { // Isn't modeled correctly (since it's based on DutyCycle logic), try to find better logic
                this.currentVelocity = (1 / this.timeForOne) * this.lastCommand.demand;
                this.currentPosition += (time - this.lastUpdateTime) * this.currentVelocity;
                break;
            }
            case Velocity: {
                this.currentVelocity = this.lastCommand.demand;
                this.currentPosition += (time - this.lastUpdateTime) * this.currentVelocity;
                break;
            }
            case Position: {
                // https://www.desmos.com/calculator/euno0nwy5h
                double t = time - this.lastCommand.startTimestamp;

                if (Math.abs(lastCommand.demand - this.currentPosition) < 0.01) {
                    this.currentPosition = lastCommand.demand;
                    break;
                }

                double a = this.lastCommand.demand - this.lastCommand.initialPosition;
                double b = Math.abs(a) * (1 / this.timeConstantFactor);

                double e = Math.pow(Math.E, -t/b);

                double r = (Math.abs(a) * (1 - e));

                this.currentPosition = (a < 0 ? -r : r) + this.lastCommand.initialPosition;
                this.currentVelocity = (this.currentPosition - this.lastPosition) - (time - this.lastUpdateTime);

                break;
            }
        }

        this.lastPosition = this.currentPosition;
        this.lastUpdateTime = time;
    }

    public double getPosition() {
        return (this.currentPosition * this.CPR);
    }

    public double getVelocity() {
        return (this.currentVelocity * this.CPR) / 60;
    }

    public void setEncoderPosition(double position) {
        this.currentPosition = position / this.CPR;
    }

    private class Setpoint {
        public final double startTimestamp;
        public final MotorControlMode controlMode;
        public final double demand;
        public final double initialPosition;

        public final double journeyDuration;

        public Setpoint(double t, MotorControlMode m, double d, double i) {
            this.startTimestamp = t;
            this.controlMode = m;
            this.demand = d;
            this.initialPosition = i;

            if (this.controlMode == MotorControlMode.Position) {
                this.journeyDuration = timeForOne * (d - i);
            } else {
                this.journeyDuration = 0;
            }
        }
    }

    public enum MotorControlMode {
        DutyCycle,
        Voltage,
        Current,
        Position,
        Velocity
    }
}