package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Grabber;

import static frc.robot.autos.primitives.RunGrabberIntakeAnyDirection.IntakeDirection.CONSUME;

public class ConsumeGamePieceCommand extends SequentialCommandGroup {

    public ConsumeGamePieceCommand(Grabber grabber, double maxTimeout) {
        RunGrabberIntakeAnyDirection intakeGamePiece = new RunGrabberIntakeAnyDirection(grabber, maxTimeout, CONSUME);

        addCommands(intakeGamePiece);
    }
}
