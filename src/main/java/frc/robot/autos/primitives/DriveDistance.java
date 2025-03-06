package frc.robot.autos.primitives;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveDistance extends SequentialCommandGroup {
    public DriveDistance(CommandSwerveDrivetrain commandSwerveDrivetrain, DriveDirection direction, double distance, DistanceUnit units) {
        MonitorDistanceDriven monitorDriveDistance = new MonitorDistanceDriven(commandSwerveDrivetrain, direction, distance, units);
        Command doIt = Commands.sequence(
            Commands.deadline(monitorDriveDistance, monitorDriveDistance.getActualDrivetrainCommand()),
            commandSwerveDrivetrain.runOnce(() -> new SwerveRequest.SwerveDriveBrake())
        );

        addCommands(doIt);
    }
}
