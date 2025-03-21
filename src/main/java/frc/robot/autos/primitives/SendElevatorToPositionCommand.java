package frc.robot.autos.primitives;

import frc.robot.StallOnInit;
import frc.robot.subsystems.Elevator;

public class SendElevatorToPositionCommand extends DurationCommand {

    private final Elevator elevator;
    private final double desiredElevatorPosition;
    private final StallOnInit stallOnInit;
    private int atDesiredPositionCount;

    private static final double CLOSE_EPSILON = 4.0;  // FIXME: What are the position values???? raw ticks, inches, something....
    private static final double EPSILON = 0.1;  // FIXME: What kind of slack will we accept for "at desired position"

    public SendElevatorToPositionCommand(Elevator elevator, double maxTimeout, double desiredElevatorPosition) {
        this(elevator, maxTimeout, desiredElevatorPosition, null);
    }

    public SendElevatorToPositionCommand(Elevator elevator, double maxTimeout, double desiredElevatorPosition, StallOnInit stallOnInit) {
        super(maxTimeout);
        this.elevator = elevator;
        this.desiredElevatorPosition = desiredElevatorPosition;
        this.stallOnInit = stallOnInit;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.atDesiredPositionCount = 0;
    }

    @Override
    public void execute() {
        super.execute();

        double currentElevatorPosition = elevator.getEncoderPosition();
        double diff = desiredElevatorPosition - currentElevatorPosition;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            elevator.stall();
            ++atDesiredPositionCount;
        } else {
            atDesiredPositionCount = 0;
            if (diff > 0) {
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
        elevator.stall();
        super.end(interrupted);

        if (interrupted) {
            elevator.stop();
        } else {
            elevator.stall();
        }

        if (!interrupted && stallOnInit != null) {
            stallOnInit.setStallOnInit(true);
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || atDesiredPositionCount > 3;
    }
}
