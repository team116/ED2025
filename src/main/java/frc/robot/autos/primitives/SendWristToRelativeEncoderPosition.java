package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Wrist;

public class SendWristToRelativeEncoderPosition extends DurationCommand {

    private final Wrist wrist;
    private final double desiredWristPosition;
    private boolean atDesiredPosition;

    private static final double EPSILON = 0.5;  // UGH.  Seems like 0.42857 degrees per click is what we get at wrist itself

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

        SmartDashboard.putNumber("curr wrist", currentWristPosition);
        SmartDashboard.putNumber("diff wrist", diff);

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            wrist.stop();
            atDesiredPosition = true;
        } else {
            if (diff < 0) {
                wrist.upSlow();
            } else {
                wrist.downSlow();
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
