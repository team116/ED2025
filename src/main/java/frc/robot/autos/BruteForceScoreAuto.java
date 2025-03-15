package frc.robot.autos;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDirection;
import frc.robot.autos.primitives.DriveDistance;
import frc.robot.autos.primitives.ExpelGamePieceCommand;
import frc.robot.autos.primitives.HoldWristAtRelativeAngle;
import frc.robot.autos.primitives.MoveWrist;
import frc.robot.autos.primitives.SendElevatorToPositionCommand;
import frc.robot.autos.primitives.SendWristToRelativeEncoderAngle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class BruteForceScoreAuto extends SequentialCommandGroup {
    public BruteForceScoreAuto(CommandSwerveDrivetrain commandSwerveDrivetrain, Elevator elevator, Intake intake, Wrist wrist) {

        // NOTE: 88" from start line to front face of reef  (30 inch robot, 3" bumpers - only account for front bumps)
        DriveDistance driveForward = new DriveDistance(commandSwerveDrivetrain, DriveDirection.FORWARD, 45, Units.Inches);
        MoveWrist moveWristDown = new MoveWrist(wrist, 1.0, true);
        // SendWristToRelativeEncoderAngle wristOutAngled = new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_TROUGH_LEVEL_FOR_AUTO);
        // SendWristToRelativeEncoderAngle wristUp = new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_TROUGH_LEVEL_FOR_AUTO);
        ExpelGamePieceCommand coralToTrough = new ExpelGamePieceCommand(intake, 0.5);
        MoveWrist moveWristUp = new MoveWrist(wrist, 1.0, false);

        addCommands(
            driveForward,
            moveWristDown,
            coralToTrough,
            moveWristUp
        );
    }
}
