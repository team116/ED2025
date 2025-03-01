package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;

import static frc.robot.autos.primitives.RunIntakeAnyDirection.IntakeDirection.CONSUME;

public class ConsumeGamePieceCommand extends SequentialCommandGroup {

    public ConsumeGamePieceCommand(Intake intake, double maxTimeout) {
        RunIntakeAnyDirection intakeGamePiece = new RunIntakeAnyDirection(intake, maxTimeout, CONSUME);

        addCommands(intakeGamePiece);
    }
}
