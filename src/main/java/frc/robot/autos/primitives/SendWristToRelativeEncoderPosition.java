package frc.robot.autos.primitives;

import frc.robot.subsystems.Grabber;

public class SendWristToRelativeEncoderPosition extends DurationCommand {

    // FIXME: Actually move write to position based upon relative 
    private final Grabber grabber;
    private final double desiredWristPosition;
    private boolean atDesiredPosition;

    private static final double EPSILON = 0.1;  // FIXME: What kind of slack will we accept for "at desired position"

    public SendWristToRelativeEncoderPosition(Grabber grabber, double maxTimeout, double position) {
        super(maxTimeout);
        this.grabber = grabber;
        this.desiredWristPosition = position;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.atDesiredPosition = false;
    }

    @Override
    public void execute() {
        super.execute();

        double currentWristPosition = grabber.getWristRelativePosition();
        double diff = desiredWristPosition - currentWristPosition;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            grabber.stopWristMotors();
            atDesiredPosition = true;
        } else {
            if (diff < 0) {
                grabber.wristUp();
            } else {
                grabber.wristDown();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stopWristMotors();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || atDesiredPosition;
    }
}
