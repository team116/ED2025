package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class DefaultElevatorCommand extends Command {

    private final Elevator elevator;
    private final Joystick gunnerLogitech;

    public DefaultElevatorCommand(Elevator elevator, Joystick gunnerLogitech) {
        this.elevator = elevator;
        this.gunnerLogitech = gunnerLogitech;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stop();
    }

    @Override
    public void execute() {
        super.execute();
        elevator.move(withDeadband(-gunnerLogitech.getY()));
    }

    @Override
    public void end(boolean interrupted){
        elevator.stop();
    }

    private double withDeadband(double input) {
        if (input > -0.02d && input < 0.02d) {
            return 0.0d;
        }
        return input;
    }
}
