package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

public class DefaultClimberCommand extends Command {

    private final Climber climber;
    private final Joystick gunnerPad;

    public DefaultClimberCommand(Climber climber, Joystick gunnerPad) {
        this.climber = climber;
        this.gunnerPad = gunnerPad;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.stop();
    }

    @Override
    public void execute() {
        super.execute();
        
        if (gunnerPad.getRawButton(14)) {
            climber.pullUp();
        } else if (gunnerPad.getRawButton(10)) {
            climber.dropDown();
        } else {
            climber.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        climber.stop();
    }
}
