package frc.robot.autos;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.primitives.DriveDirection;
import frc.robot.autos.primitives.DriveDistance;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveOffTheLine extends SequentialCommandGroup {

    public DriveOffTheLine(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        DriveDistance driveForward = new DriveDistance(commandSwerveDrivetrain, DriveDirection.FORWARD, 18, Units.Inches);

        addCommands(driveForward);
    }
    
}
