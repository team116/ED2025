package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.StallOnInit;
import frc.robot.subsystems.Elevator;

public class DefaultElevatorCommand extends Command implements StallOnInit {

    private final Elevator elevator;
    private final Joystick gunnerLogitech;
    private final Joystick gunnerPad;

    private boolean moveRequested;
    private double desiredPosition;
    private boolean stallMotors;
    private boolean stallOnInit = false;

    private static final boolean HOLD_AT_POSITION = false;
    private static final double EPSILON = 1.0;

    public DefaultElevatorCommand(Elevator elevator, Joystick gunnerLogitech, Joystick gunnerPad) {
        this.elevator = elevator;
        this.gunnerLogitech = gunnerLogitech;
        this.gunnerPad = gunnerPad;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        if (stallOnInit) {
            elevator.stall();
        } else {
            elevator.stop();
        }
        stallMotors = stallOnInit;
        moveRequested = false;
        desiredPosition = elevator.getEncoderPosition();
        SmartDashboard.putBoolean("elevator stall", stallMotors);
    }

    @Override
    public void execute() {
        super.execute();
        checkStallMotors();

        double adjustedWithDeadBand = withDeadband(shape(-gunnerLogitech.getY()));
        //elevator.move(withDeadband(-gunnerLogitech.getY()));
        if (gunnerPad.getRawButton(5)) {
            adjustedWithDeadBand = -0.195d;
        }
        if (gunnerPad.getRawButton(6)) {
            adjustedWithDeadBand = 0.011d;
        }

        SmartDashboard.putNumber("elevator power", adjustedWithDeadBand);

        if (adjustedWithDeadBand >= 0.01d || adjustedWithDeadBand <= -0.01d) {
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
            stallOnInit = true;
        } else if (gunnerLogitech.getPOV() == 90) {
            SmartDashboard.putBoolean("elevator stall", false);
            stallMotors = false;
            stallOnInit = false;
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

    public void setStallOnInit(boolean doStall) {
        stallOnInit = doStall;
    }
}
