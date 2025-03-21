package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DesiredAngleCallback;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class DefaultWristCommand extends Command implements DesiredAngleCallback {

    private final Wrist wrist;
    private final Joystick gunnerLogitech;
    private boolean stallMotors;
    private boolean moveRequested;
    private double desiredWristAngle;

    private static final boolean HOLD_AT_ANGLE = true; // FIXME: turn this back on once we know that we have angles correct
    private static final double EPSILON = 1.0;

    public DefaultWristCommand(Wrist wrist, Joystick gunnerLogitech) {
        this.wrist = wrist;
        this.gunnerLogitech = gunnerLogitech;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        wrist.stop();
        stallMotors = true;
        moveRequested = false;
        desiredWristAngle = wrist.getRelativeAngle();
    }

    @Override
    public void execute() {
        super.execute();
        checkStallMotors();

        //double manualMoveRequest = gunnerLogitech.getRawAxis(3);
        
        if (gunnerLogitech.getRawButton(9)) {
            wrist.up();  // FIXME: Put this back
            //wrist.setPower(Math.min(manualMoveRequest, 0));
            moveRequested = true;
        } else if (gunnerLogitech.getRawButton(10)) {
            //wrist.setPower(Math.max(0, manualMoveRequest));
            wrist.down();  // FIXME: Put this back
            moveRequested = true;
        } else {
            if (moveRequested) {
                desiredWristAngle = wrist.getRelativeAngle();
            }
            
            moveRequested = false;
        }

        //SmartDashboard.putNumber("wrist power", manualMoveRequest);

        if (!moveRequested) {
            if (HOLD_AT_ANGLE) {
                holdAtAngle();
            } else {
                if (stallMotors) {
                    wrist.stall();  // FIXME: Should put in something that knows "desired" position and tries to keep it there
                } else {
                    wrist.stop();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted){
        wrist.stop();
    }

    private void checkStallMotors() {
        if (gunnerLogitech.getPOV() == 0) {
            stallMotors = true;
        } else if (gunnerLogitech.getPOV() == 180) {
            stallMotors = false;
        }
    }

    private void holdAtAngle() {
        double currentWristAngle = wrist.getRelativeAngle();
        double diff = desiredWristAngle - currentWristAngle;

        double absDiff = Math.abs(diff);
        if (absDiff < EPSILON) {
            if (stallMotors) {
                wrist.stall();
            } else {
                wrist.stop();
            }
        } else {
            if (diff < 0) {
                wrist.upSlow();
            } else {
                wrist.downSlow();
            }
        }
    }

    public void setDesiredAngle(double desiredAngle) {
        this.desiredWristAngle = desiredAngle;
    }
}
