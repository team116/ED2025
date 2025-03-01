package frc.robot.autos;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDirection;
import frc.robot.autos.primitives.DriveDistance;
import frc.robot.autos.primitives.ExpelGamePieceCommand;
import frc.robot.autos.primitives.SendElevatorToPositionCommand;
import frc.robot.autos.primitives.SendWristToAbsoluteEncoderAngle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class ScoreTroughCenter extends SequentialCommandGroup {
    
    public ScoreTroughCenter(CommandSwerveDrivetrain commandSwerveDrivetrain, Elevator elevator, Intake intake, Wrist wrist) {

        DriveDistance driveForward = new DriveDistance(commandSwerveDrivetrain, DriveDirection.FORWARD, 48, Units.Inches);
        SendElevatorToPositionCommand elevatorUp = new SendElevatorToPositionCommand(elevator, 3.0, 176.0);
        SendWristToAbsoluteEncoderAngle wristOutStraight = new SendWristToAbsoluteEncoderAngle(wrist, 2.0, 0.0);
        ExpelGamePieceCommand coralToTrough = new ExpelGamePieceCommand(intake, 2.0d);
        
        addCommands(Commands.sequence(
            wristOutStraight,
            elevatorUp,
            driveForward,
            coralToTrough
        ));
    }
}
