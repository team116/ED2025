package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class DefaultIntakeCommand extends Command {

    private final Intake intake;
    private final Joystick gunnerLogitech;
    private final Joystick gunnerPad;

    public DefaultIntakeCommand(Intake intake, Joystick gunnerLogitech, Joystick gunnerPad) {
        this.intake = intake;
        this.gunnerLogitech = gunnerLogitech;
        this.gunnerPad = gunnerPad;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stop();
    }

    @Override
    public void execute() {
        super.execute();
        /*
        if (gunnerLogitech.getRawButton(1)) {
            intake.consume();
        } else if (gunnerLogitech.getRawButton(2)) {
            intake.expel();
        } else {
            intake.stop();
        }
        */

        if (gunnerLogitech.getRawButtonPressed(1)) {
            SmartDashboard.putString("Intake Mode", "Consume");
            intake.consume();
        } else if (gunnerLogitech.getRawButtonPressed(2)) {
            if (gunnerLogitech.getRawAxis(3) < 0) {
                SmartDashboard.putString("Intake Mode", "Launch");
                intake.launch();
            } else {
                SmartDashboard.putString("Intake Mode", "Expel");
                intake.expel();
            }
        } else if (gunnerLogitech.getRawButtonPressed(4)) {
            SmartDashboard.putString("Intake Mode", "OFF");
            intake.stop();
        } else if (gunnerPad.getRawButton(12)) {
            SmartDashboard.putString("Intake Mode", "Super Launch");
            intake.superLaunch();
        } else if (gunnerPad.getRawButton(2)) {
            SmartDashboard.putString("Intake Mode", "OFF");
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        intake.stop();
    }
}

