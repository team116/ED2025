package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Grabber;

import static frc.robot.autos.primitives.RunGrabberIntakeAnyDirection.IntakeDirection.EXPEL;

public class ExpelGamePieceCommand extends SequentialCommandGroup {

    public ExpelGamePieceCommand(Grabber grabber, double maxTimeout) {
        RunGrabberIntakeAnyDirection expelGamePiece = new RunGrabberIntakeAnyDirection(grabber, maxTimeout, EXPEL);

        addCommands(expelGamePiece);
    }
}
