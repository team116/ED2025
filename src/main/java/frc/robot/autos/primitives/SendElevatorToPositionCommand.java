package frc.robot.autos.primitives;

import frc.robot.subsystems.Elevator;

public class SendElevatorToPositionCommand extends DurationCommand {

    private final Elevator elevator;
    private final double desiredElevatorPosition;
    private boolean atDesiredPosition;

    private static final double CLOSE_EPSILON = 5.0;  // FIXME: What are the position values???? raw ticks, inches, something....
    private static final double EPSILON = 0.1;  // FIXME: What kind of slack will we accept for "at desired position"

    public SendElevatorToPositionCommand(Elevator elevator, double maxTimeout, double desiredElevatorPosition) {
        super(maxTimeout);
        this.elevator = elevator;
        this.desiredElevatorPosition = desiredElevatorPosition;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.atDesiredPosition = false;
    }

    @Override
    public void execute() {
        super.execute();

        double currentElevatorPosition = elevator.getEncoderPosition();
        double diff = desiredElevatorPosition - currentElevatorPosition;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            elevator.stop();
            atDesiredPosition = true;
        } else {
            if (diff < 0) {
                if (absDiff < CLOSE_EPSILON) {
                    elevator.moveDownSlow();
                } else {
                    elevator.moveDown();
                }
            } else {
                if (absDiff < CLOSE_EPSILON) {
                    elevator.moveUpSlow();
                } else {
                    elevator.moveUp();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || atDesiredPosition;
    }
}
