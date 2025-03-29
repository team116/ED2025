package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.autos.primitives.DriveDirection;
import frc.robot.autos.primitives.DriveDistance;
import frc.robot.autos.primitives.DurationCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static frc.robot.Constants.LIMELIGHT_NAME;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AprilTagAutoAlign extends SequentialCommandGroup{

    private static final double OUTER_EPSILON = 10.0d;
    private static final double EPSILON = 2.0d;

    private final SwerveRequest.RobotCentric strafeOnly = new SwerveRequest.RobotCentric()
        .withRotationalRate(0.0d)
        .withVelocityX(0.0d)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AprilTagAutoAlign(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, true);
    }

    public AprilTagAutoAlign(CommandSwerveDrivetrain drivetrain, boolean driveRight)  {
        
        final LimelightMonitor limelightMonitor = new LimelightMonitor();
        Command strafeToAlign = drivetrain.applyRequest(() -> strafeOnly.withVelocityY(limelightMonitor.getVelocity()));
        Command driveToFinalPosition = Commands.none();   //  new DriveDistance(drivetrain, driveRight ? DriveDirection.RIGHT : DriveDirection.LEFT, 6.5, Units.Inches);
        addCommands(
            Commands.sequence(
                Commands.deadline(limelightMonitor, strafeToAlign),
                Commands.either(
                    driveToFinalPosition, 
                    Commands.none(), 
                    () -> limelightMonitor.stabilized()
                )
            )
        );
    }

    private static class LimelightMonitor extends DurationCommand {

        private int stabilizedCount;
        private double xVelocityMetersPerSecond;
        private boolean isActivelyMoving;
        private double startTime;

        public LimelightMonitor() {
            super(1.2d); 
        }

        @Override
        public void initialize() {
            super.initialize();
            stabilizedCount = 0;
            xVelocityMetersPerSecond = 0.0;
            isActivelyMoving = false;
            startTime = Timer.getFPGATimestamp();
        }

        @Override
        public void execute() {
            super.execute();
            double offsetX = LimelightHelpers.getTX(Constants.LIMELIGHT_NAME);
            if (stillNeedToMove(offsetX)) {
                if (!isActivelyMoving) {
                    isActivelyMoving = true;
                    stabilizedCount = 0;
                    xVelocityMetersPerSecond = 0.5;
                    if (Math.abs(offsetX) < OUTER_EPSILON) {
                        xVelocityMetersPerSecond = 0.25;
                    }
                    if (offsetX > 0) {
                        xVelocityMetersPerSecond = -xVelocityMetersPerSecond;
                    }
                }
            } else { 
                isActivelyMoving = false;
                xVelocityMetersPerSecond = 0.0;
                stabilizedCount++;
            }
            SmartDashboard.putNumber("Stabilized Count",stabilizedCount);
            SmartDashboard.putNumber("x Velocity",xVelocityMetersPerSecond);
            SmartDashboard.putBoolean("Is Actively Moving", isActivelyMoving);

        }

        @Override
        public boolean isFinished() {
            return (super.isFinished() || stabilized());
        }

        @Override
        public void end(boolean interrupted) {
            SmartDashboard.putNumber("Time taken for apriltag",Timer.getFPGATimestamp() - startTime);
        }

        public boolean stillNeedToMove(double offsetX) {
            SmartDashboard.putBoolean("TV",LimelightHelpers.getTV(LIMELIGHT_NAME));
            SmartDashboard.putNumber("Offset X", offsetX);
            return LimelightHelpers.getTV(LIMELIGHT_NAME) && (Math.abs(offsetX) > EPSILON);
        }

        public double getVelocity() {
            return xVelocityMetersPerSecond;
        }

        public boolean stabilized() {
            return stabilizedCount > 3;
        }
    }
}
