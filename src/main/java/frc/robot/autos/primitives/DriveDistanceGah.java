package frc.robot.autos.primitives;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDistance.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveDistanceGah extends SequentialCommandGroup {
    public DriveDistanceGah(CommandSwerveDrivetrain commandSwerveDrivetrain, Direction direction, double distance, DistanceUnit units) {
        DriveDistance driveDistance = new DriveDistance(commandSwerveDrivetrain, direction, distance, units);
        Command doIt = Commands.sequence(
            Commands.deadline(driveDistance, driveDistance.getActualDrivetrainCommand()),
            commandSwerveDrivetrain.runOnce(() -> new SwerveRequest.SwerveDriveBrake())
        );

        addCommands(doIt);
    }
}
