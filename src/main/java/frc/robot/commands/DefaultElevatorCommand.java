package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DefaultElevatorCommand extends Command {

    private final Elevator elevator;
    private final Joystick gunnerLogitech;

    private boolean moveRequested;
    private double desiredPosition;
    private boolean stallMotors;

    private static final boolean HOLD_AT_POSITION = false;
    private static final double EPSILON = 1.0;

    public DefaultElevatorCommand(Elevator elevator, Joystick gunnerLogitech) {
        this.elevator = elevator;
        this.gunnerLogitech = gunnerLogitech;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stop();
        stallMotors = false;
        moveRequested = false;
        desiredPosition = elevator.getEncoderPosition();
        SmartDashboard.putBoolean("elevator stall", false);
    }

    @Override
    public void execute() {
        super.execute();
        checkStallMotors();

        double adjustedWithDeadBand = withDeadband(shape(-gunnerLogitech.getY()));
        //elevator.move(withDeadband(-gunnerLogitech.getY()));

        SmartDashboard.putNumber("elevator power", adjustedWithDeadBand);

        if (adjustedWithDeadBand >= 0.02d || adjustedWithDeadBand <= -0.02d) {
            moveRequested = true;
            elevator.move(adjustedWithDeadBand);
        } else {
            if (moveRequested) {
                desiredPosition = elevator.getEncoderPosition();
            }

            moveRequested = false;
        }

        if (!moveRequested) {
            if (HOLD_AT_POSITION) {
                holdAtPosition();
            } else {
                if (stallMotors) {
                    elevator.stall();  // FIXME: Should put in something that knows "desired" position and tries to keep it there
                } else {
                    elevator.stop();
                }
            }
        }
    }

    private void checkStallMotors() {
        if (gunnerLogitech.getPOV() == 270) {
            SmartDashboard.putBoolean("elevator stall", true);
            stallMotors = true;
        } else if (gunnerLogitech.getPOV() == 90) {
            SmartDashboard.putBoolean("elevator stall", false);
            stallMotors = false;
        }
    }

    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }

    public void setDesiredPosition(double desiredPosition) {
        this.desiredPosition = desiredPosition;
    }

    public void holdAtPosition() {
        double currentPosition = elevator.getEncoderPosition();
        double diff = desiredPosition - currentPosition;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            if (stallMotors) {
                elevator.stall();
            } else {
                elevator.stop();
            }
        } else {
            if (diff < 0) {
                elevator.moveUpSlow();
            } else {
                elevator.moveDownSlow();
            }
        }
    }

    private double withDeadband(double input) {
        if (input > -0.02d && input < 0.02d) {
            return 0.0d;
        }
        return input;
    }

    // NOTE: Positive value should be "DOWN" for elevator.
    private double shape(double input) {

        if (input > 0.0d) {
            input *= input;
        } else {
            input *= -input;
        }

        if (input > 0.2d) {
            input /= 4.0;
        }

        return input;
    }
}
