package frc.robot.autos.primitives;

import java.net.Authenticator.RequestorType;

import frc.robot.DesiredAngleCallback;
import frc.robot.subsystems.Wrist;

// NOTE: This will ONLY exit when the duration has expired, otherwise continues to hold the full time
public class HoldWristAtRelativeAngle extends DurationCommand implements DesiredAngleCallback {

    private final Wrist wrist;

    private boolean stallMotors;
    private double desiredWristAngle;

    private static final double EPSILON = 1.0;

    public HoldWristAtRelativeAngle(Wrist wrist, double duration, double wristAngle) {
        super(duration);
        
        this.wrist = wrist;
        this.desiredWristAngle = wristAngle;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        //wrist.stop();
        stallMotors = true;
    }

    @Override
    public void execute() {
        super.execute();
        holdAtAngle();
    }

    @Override
    public void end(boolean interrupted){
        //wrist.stop();
        wrist.stall();
    }

    private void holdAtAngle() {
        double currentWristAngle = wrist.getRelativeAngle();
        double diff = desiredWristAngle - currentWristAngle;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            if (stallMotors) {
                wrist.stall();
            } else {
                wrist.stop();
            }
        } else {
            if (diff < 0) {
                wrist.upSlow();
            } else {
                wrist.downSlow();
            }
        }
    }

    public void setDesiredAngle(double desiredAngle) {
        this.desiredWristAngle = desiredAngle;
    }
}
