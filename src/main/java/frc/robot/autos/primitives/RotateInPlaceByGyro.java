package frc.robot.autos.primitives;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateInPlaceByGyro extends SequentialCommandGroup {
    private static final int PIGEON_DEVICE_ID = 7 ;

    private final Pigeon2 pigeon = new Pigeon2(PIGEON_DEVICE_ID);
    private final CommandSwerveDrivetrain drivetrain;
    private final double desiredAngle;

    public RotateInPlaceByGyro(CommandSwerveDrivetrain drivetrain, double desiredAngle) {
        this.desiredAngle = desiredAngle;
        this.drivetrain = drivetrain;
    }
}
