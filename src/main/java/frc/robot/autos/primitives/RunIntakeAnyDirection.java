package frc.robot.autos.primitives;

import frc.robot.subsystems.Intake;

public class RunIntakeAnyDirection extends DurationCommand {

    public enum IntakeDirection {
        CONSUME,
        EXPEL
    }

    private final Intake intake;
    private final IntakeDirection direction;

    public RunIntakeAnyDirection(Intake intake, double maxTimeout, IntakeDirection direction) {
        super(maxTimeout);
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        super.initialize();
        if (direction == IntakeDirection.CONSUME) {
            intake.consume();
        } else {
            intake.expel();
        } 
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
