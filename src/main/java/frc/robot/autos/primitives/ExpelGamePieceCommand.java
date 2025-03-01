package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

import static frc.robot.autos.primitives.RunIntakeAnyDirection.IntakeDirection.EXPEL;

public class ExpelGamePieceCommand extends SequentialCommandGroup {

    public ExpelGamePieceCommand(Intake intake, double maxTimeout) {
        RunIntakeAnyDirection expelGamePiece = new RunIntakeAnyDirection(intake, maxTimeout, EXPEL);

        addCommands(expelGamePiece);
    }
}
