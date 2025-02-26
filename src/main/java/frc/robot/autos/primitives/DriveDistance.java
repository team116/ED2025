package frc.robot.autos.primitives;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveDistance extends DurationCommand {

    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final double distance;
    private final Direction direction;
    private final boolean[] reachedPositionForModule;
    private final double xDirectionVelocity;
    private final double yDirectionVelocity;
    private final SwerveRequest.RobotCentric driveDirection = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final double WORST_CASE_METERS_PER_SECOND = Units.Inches.toBaseUnits(8.0d);
    private static final double METERS_AWAY_FROM_DESIRED_THRESHOLD = 0.05;
    private static final double SPEED_METERS_PER_SECOND = (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) * 0.1d;  // 30% max speed

    public enum Direction {
        FORWARD(1.0d, 0.0d),
        REVERSE(-1.0d, 0.0d),
        LEFT(0.0d, 1.0d),
        RIGHT(0.0d, -1.0d),
        DIAGONAL_FORWARD_LEFT(1.0d, 1.0d),
        DIAGONAL_FORWARD_RIGHT(1.0d, -1.0d),
        DIAGONAL_BACKWARD_LEFT(-1.0d, 1.0d),
        DIAGONAL_BACKWARD_RIGHT(-1.0d, -1.0d);

        public final double xRatio;
        public final double yRatio;

        private Direction(double xRatio, double yRatio) {
            this.xRatio = xRatio;
            this.yRatio = yRatio;
        }

        public Direction getInverse() {
            switch(this) {
                case FORWARD: return REVERSE;
                case REVERSE: return FORWARD;
                case LEFT:    return RIGHT;
                case RIGHT:   return LEFT;
                case DIAGONAL_FORWARD_LEFT: return DIAGONAL_BACKWARD_RIGHT;
                case DIAGONAL_BACKWARD_RIGHT: return DIAGONAL_FORWARD_LEFT;
                case DIAGONAL_FORWARD_RIGHT: return DIAGONAL_BACKWARD_LEFT;
                case DIAGONAL_BACKWARD_LEFT: return DIAGONAL_FORWARD_RIGHT;
                default: return REVERSE;
            }
        }
    }

    public DriveDistance(CommandSwerveDrivetrain commandSwerveDrivetrain, Direction direction, double distance, DistanceUnit units) {
        super(deriveMaxTimeoutFromDistance(distance, units));
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.distance = units.toBaseUnits(distance);
        this.direction = direction;
        this.reachedPositionForModule = new boolean[commandSwerveDrivetrain.getModules().length];

        this.xDirectionVelocity = direction.xRatio * SPEED_METERS_PER_SECOND;
        this.yDirectionVelocity = direction.yRatio * SPEED_METERS_PER_SECOND;

        // FIXME: Believe that this should be handled by getActualDrivetrainCommand() command
        //addRequirements(commandSwerveDrivetrain);
    }

    public Command getActualDrivetrainCommand() {
        return commandSwerveDrivetrain.applyRequest(() -> driveDirection
            .withVelocityX(xDirectionVelocity)
            .withVelocityY(yDirectionVelocity)
            .withRotationalRate(0.0d));
    }

    @Override
    public void initialize() {
        super.initialize();
        int moduleNum = 0;
        for (SwerveModule swerveModule : commandSwerveDrivetrain.getModules()) {
            swerveModule.resetPosition();
            SmartDashboard.putNumber("initialDistance" + moduleNum, swerveModule.getPosition(true).distanceMeters);
            reachedPositionForModule[moduleNum++] = false;
        }

        SmartDashboard.putNumber("desiredDistance", distance);
        SmartDashboard.putNumber("xVelocity", xDirectionVelocity);
        SmartDashboard.putNumber("yVelocity", yDirectionVelocity);
        SmartDashboard.putBooleanArray("modulesAtPosition", reachedPositionForModule);
    }

    @Override
    public void execute() {
        super.execute();

        int moduleNum = 0;
        for (SwerveModule swerveModule : commandSwerveDrivetrain.getModules()) {
            double currentDistance = swerveModule.getPosition(true).distanceMeters;

            SmartDashboard.putNumber("currentDistance" + moduleNum, currentDistance);
            if (Math.abs(currentDistance - distance) < METERS_AWAY_FROM_DESIRED_THRESHOLD) {
                reachedPositionForModule[moduleNum] = true;
            }

            ++moduleNum;
        }

        SmartDashboard.putBooleanArray("modulesAtPosition", reachedPositionForModule);
        SmartDashboard.putString("runState", "EXEC");
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("runState", "END");
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
          // Only exit after all are at positions, or timer hits timeout
        return (allAtFinalPositions() || super.isFinished());
    }

    private boolean allAtFinalPositions() {
        // NOTE: This implemenation is NOT using run to position, so this could fail easily :-(
        // FIXME: See if we can do a run to position using the phoenix stuff
        boolean foundAllPositions = true;
        for (boolean moduleAtPosition : reachedPositionForModule) {
            foundAllPositions &= moduleAtPosition;
        }
 
        SmartDashboard.putBoolean("atFinalPositions", foundAllPositions);
        return foundAllPositions;
    }

    private static double deriveMaxTimeoutFromDistance(double distance, DistanceUnit units) {
        // NOTE: All times should be non-negative
        //return Math.abs(units.toBaseUnits(distance) / WORST_CASE_METERS_PER_SECOND);
        // FIXME: Just to rule this out
        return 2.0;
    }
}
