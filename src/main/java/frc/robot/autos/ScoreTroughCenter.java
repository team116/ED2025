package frc.robot.autos;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDirection;
import frc.robot.autos.primitives.DriveDistance;
import frc.robot.autos.primitives.ExpelGamePieceCommand;
import frc.robot.autos.primitives.HoldWristAtRelativeAngle;
import frc.robot.autos.primitives.SendElevatorToPositionCommand;
import frc.robot.autos.primitives.SendWristToRelativeEncoderAngle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ScoreTroughCenter extends SequentialCommandGroup {
    
    public ScoreTroughCenter(CommandSwerveDrivetrain commandSwerveDrivetrain, Elevator elevator, Intake intake, Wrist wrist) {

        DriveDistance driveForward = new DriveDistance(commandSwerveDrivetrain, DriveDirection.FORWARD, 48, Units.Inches);
        SendElevatorToPositionCommand elevatorUp = new SendElevatorToPositionCommand(elevator, 3.0, Elevator.LEVEL_1_POSITION);
        SendWristToRelativeEncoderAngle wristOutStraight = new SendWristToRelativeEncoderAngle(wrist, 2.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE);
        HoldWristAtRelativeAngle holdWristAtAngle = new HoldWristAtRelativeAngle(wrist, 10.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE);
        ExpelGamePieceCommand coralToTrough = new ExpelGamePieceCommand(intake, 2.0d);
        
        addCommands(
            wristOutStraight,
            Commands.deadline(
                Commands.sequence(
                    elevatorUp,
                    driveForward,
                    coralToTrough
                ),
                holdWristAtAngle
            )
        );
    }
}
