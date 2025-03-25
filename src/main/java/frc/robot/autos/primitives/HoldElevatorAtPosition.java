package frc.robot.autos.primitives;

import frc.robot.subsystems.Elevator;

public class HoldElevatorAtPosition extends DurationCommand {

    private final Elevator elevator;
    private double desiredElevatorPosition;
    private boolean stallMotors = true;

    private static final double CLOSE_EPSILON = 4.0;  // FIXME: What are the position values???? raw ticks, inches, something....
    private static final double EPSILON = 0.1;

    public HoldElevatorAtPosition(Elevator elevator, double duration, double position) {
        super(duration);
        
        this.elevator = elevator;
        this.desiredElevatorPosition = position;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stall();
    }

    @Override
    public void execute() {
        super.execute();
        holdAtPosition();
    }

    @Override
    public void end(boolean interrupted){
        elevator.stall();
    }

    public void holdAtPosition() {
        double currentElevatorPosition = elevator.getEncoderPosition();
        double diff = desiredElevatorPosition - currentElevatorPosition;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            elevator.stall();
        } else {
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
}
