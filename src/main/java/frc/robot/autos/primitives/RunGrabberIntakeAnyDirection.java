package frc.robot.autos.primitives;

import frc.robot.subsystems.Grabber;

public class RunGrabberIntakeAnyDirection extends DurationCommand {

    public enum IntakeDirection {
        CONSUME,
        EXPEL
    }

    private final Grabber grabber;
    private final IntakeDirection direction;

    public RunGrabberIntakeAnyDirection(Grabber grabber, double maxTimeout, IntakeDirection direction) {
        super(maxTimeout);
        this.grabber = grabber;
        this.direction = direction;

        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (direction == IntakeDirection.CONSUME) {
            grabber.runIntakeMotorsToConsume();
        } else {
            grabber.runIntakeMotorsToExpel();
        } 
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stopIntakeMotors();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
