package frc.robot.autos.primitives;

import frc.robot.subsystems.Wrist;

public class SendWristToAbsoluteEncoderAngle extends DurationCommand {
    
    // FIXME: Actually move write to position based upon relative 
    private final Wrist wrist;
    private final double desiredWristAngle;
    private boolean atDesiredAngle;

    private static final double ZERO_ANGLE_OFFSET = 0.0d; // FIXME: Find "horizontal" zero position

    private static final double EPSILON = 0.01;  // FIXME: What kind of slack will we accept for "at desired position"

    public SendWristToAbsoluteEncoderAngle(Wrist wrist, double maxTimeout, double angle) {
        super(maxTimeout);
        this.wrist = wrist;
        this.desiredWristAngle = angle + ZERO_ANGLE_OFFSET;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.atDesiredAngle = false;
    }

    @Override
    public void execute() {
        super.execute();

        double currentWristPosition = wrist.getAbsoluteAngle();
        double diff = desiredWristAngle - currentWristPosition;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            wrist.stop();
            atDesiredAngle = true;
        } else {
            if (diff < 0) {
                wrist.up();
            } else {
                wrist.down();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || atDesiredAngle;
    }
}
