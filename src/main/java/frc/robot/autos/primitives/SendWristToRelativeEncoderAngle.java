package frc.robot.autos.primitives;

import frc.robot.DesiredAngleCallback;
import frc.robot.commands.DefaultWristCommand;
import frc.robot.subsystems.Wrist;

public class SendWristToRelativeEncoderAngle extends DurationCommand {

    private final Wrist wrist;
    private final double desiredWristAngle;
    private final DesiredAngleCallback desiredAngleCallback;
    private boolean atDesiredAngle;

    private static final double OUTER_EPSILON = 10.0;
    private static final double EPSILON = 2.0;  // UGH.  Seems like 0.42857 degrees per click is what we get at wrist itself

    public SendWristToRelativeEncoderAngle(Wrist wrist, double maxTimeout, double angle) {
        this(wrist, maxTimeout, angle, null);
    }

    public SendWristToRelativeEncoderAngle(Wrist wrist, double maxTimeout, double angle, DesiredAngleCallback desiredAngleCallback) {
        super(maxTimeout);
        this.wrist = wrist;
        this.desiredWristAngle = angle;
        this.desiredAngleCallback = desiredAngleCallback;
    }

    @Override
    public void initialize() {
        super.initialize();
        this.atDesiredAngle = false;
    }

    @Override
    public void execute() {
        super.execute();

        double currentWristAngle = wrist.getRelativeAngle();
        double diff = desiredWristAngle - currentWristAngle;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            wrist.stall();
            atDesiredAngle = true;
        } else {
            if (diff < 0) {
                if (absDiff < OUTER_EPSILON) {
                    wrist.upSlow();
                } else {
                    wrist.up();
                }
            } else {
                if (absDiff < OUTER_EPSILON) {
                    wrist.downSlow();
                } else {
                    wrist.down();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stall();
        super.end(interrupted);

        if (!interrupted && desiredAngleCallback != null) {
            desiredAngleCallback.setDesiredAngle(desiredWristAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || atDesiredAngle;
    }
}
