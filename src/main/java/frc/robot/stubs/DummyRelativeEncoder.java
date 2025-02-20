package frc.robot.stubs;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

public class DummyRelativeEncoder implements RelativeEncoder {

    private double position = 0.0d;

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public double getVelocity() {
        return 0.0d;
    }

    @Override
    public REVLibError setPosition(double position) {
        this.position = position;
        return REVLibError.kOk;
    }
}
