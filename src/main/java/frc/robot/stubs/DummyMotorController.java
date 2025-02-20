package frc.robot.stubs;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DummyMotorController implements MotorController {

    private double speed = 0.0d;
    private boolean inverted = false;

    @Override
    public void set(double speed) {
        this.speed = speed;
    }

    @Override
    public double get() {
        return this.speed;    }

    @Override
    public void setInverted(boolean isInverted) {
        this.inverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return this.inverted;
    }

    @Override
    public void disable() {
        // DO NOTHING
    }

    @Override
    public void stopMotor() {
        this.speed = 0.0d;
    }
    
}
