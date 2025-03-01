package frc.robot.autos.primitives;

import frc.robot.subsystems.Wrist;

public class SendWristToRelativeEncoderPosition extends DurationCommand {

    // FIXME: Actually move write to position based upon relative 
    private final Wrist wrist;
    private final double desiredWristPosition;
    private boolean atDesiredPosition;

    private static final double EPSILON = 0.1;  // FIXME: What kind of slack will we accept for "at desired position"

    public SendWristToRelativeEncoderPosition(Wrist wrist, double maxTimeout, double position) {
        super(maxTimeout);
        this.wrist = wrist;
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

        double currentWristPosition = wrist.getRelativePosition();
        double diff = desiredWristPosition - currentWristPosition;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            wrist.stop();
            atDesiredPosition = true;
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
        return super.isFinished() || atDesiredPosition;
    }
}
