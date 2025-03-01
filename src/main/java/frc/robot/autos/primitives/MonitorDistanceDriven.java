package frc.robot.autos.primitives;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MonitorDistanceDriven extends DurationCommand {

    private boolean foundAllPositions;
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final SwerveModule[] swerveModules;
    private final double distanceInMeters;
    private final DriveDirection direction;
    private final boolean[] reachedPositionForModule;
    private final double[] initialPositionsInMeters;
    private final double xDirectionVelocity;
    private final double yDirectionVelocity;
    private final int numModules;
    private int currentModuleNum;
    private final SwerveRequest.RobotCentric driveDirection = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final double WORST_CASE_METERS_PER_SECOND = Units.Inches.toBaseUnits(8.0d);
    private static final double METERS_AWAY_FROM_DESIRED_THRESHOLD = 0.0075; // Third of an inch
    private static final double SPEED_METERS_PER_SECOND = (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) * 0.1d;  // 30% max speed

    public MonitorDistanceDriven(CommandSwerveDrivetrain commandSwerveDrivetrain, DriveDirection direction, double distance, DistanceUnit units) {
        super(deriveMaxTimeoutFromDistance(distance, units));
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.distanceInMeters = units.toBaseUnits(distance);
        this.direction = direction;
        this.swerveModules = commandSwerveDrivetrain.getModules();
        this.numModules = this.swerveModules.length;
        this.reachedPositionForModule = new boolean[this.numModules];
        this.initialPositionsInMeters = new double[this.numModules];

        this.xDirectionVelocity = direction.xRatio * SPEED_METERS_PER_SECOND;
        this.yDirectionVelocity = direction.yRatio * SPEED_METERS_PER_SECOND;

        // FIXME: Believe that this should be handled by getActualDrivetrainCommand() command
        //addRequirements(commandSwerveDrivetrain);
    }

    public Command getActualDrivetrainCommand() {
        // return commandSwerveDrivetrain.applyRequest(() -> driveDirection
        //     .withVelocityX(xDirectionVelocity)
        //     .withVelocityY(yDirectionVelocity)
        //     .withRotationalRate(0.0d));
        return commandSwerveDrivetrain.applyRequest(() -> driveDirection
             .withVelocityX(xDirectionVelocity)
             .withVelocityY(yDirectionVelocity)
             .withRotationalRate(0.0d));
    }

    @Override
    public void initialize() {
        super.initialize();
        int moduleNum = 0;
        for (SwerveModule swerveModule : swerveModules) {
            //swerveModule.resetPosition(); // NOTE: This seems to _NOT_ reset, so store original values to subtract
            double initialPositionInMeters = swerveModule.getPosition(true).distanceMeters;
            SmartDashboard.putNumber("initialDistance" + moduleNum, initialPositionInMeters);
            reachedPositionForModule[moduleNum] = false;
            initialPositionsInMeters[moduleNum] = initialPositionInMeters;
            ++moduleNum;
        }

        foundAllPositions = false;
        currentModuleNum = 0;

        SmartDashboard.putNumber("desiredDistance", distanceInMeters);
        SmartDashboard.putNumber("xVelocity", xDirectionVelocity);
        SmartDashboard.putNumber("yVelocity", yDirectionVelocity);
        SmartDashboard.putBooleanArray("modulesAtPosition", reachedPositionForModule);
    }

    @Override
    public void execute() {
        super.execute();

        // NOTE: Only do _ONE_ swerve module computation per execute() invocation
        if (!reachedPositionForModule[currentModuleNum]) {
            SwerveModule currentSwerveModule = swerveModules[currentModuleNum];
            //double rawCurrentDistance = swerveModule.getPosition(false).distanceMeters;

            double rawCurrentDistance = currentSwerveModule.getCachedPosition().distanceMeters;
            double currentDistanceInMeters = Math.abs(rawCurrentDistance - initialPositionsInMeters[currentModuleNum]);
            //SmartDashboard.putNumber("rawDistance" + moduleNum, rawCurrentDistance);
            //SmartDashboard.putNumber("currentDistance" + moduleNum, currentDistanceInMeters);  // NOTE: Only for testing

            double difference = Math.abs(currentDistanceInMeters - distanceInMeters);
            if (difference < METERS_AWAY_FROM_DESIRED_THRESHOLD) {
                reachedPositionForModule[currentModuleNum] = true;
            }    
        }

        currentModuleNum = ++currentModuleNum % numModules;

        //SmartDashboard.putBooleanArray("modulesAtPosition", reachedPositionForModule);
        //SmartDashboard.putString("runState", "EXEC");
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("atFinalPositions", foundAllPositions);
        SmartDashboard.putString("runState", "END");
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // Only exit after all are at positions, or timer hits timeout
        //return (allAtFinalPositions() || super.isFinished());
        return (atLeastOneAtFinalPosition() || super.isFinished());
    }

    private boolean allAtFinalPositions() {
        // NOTE: This implemenation is NOT using run to position, so this could fail easily :-(
        // FIXME: See if we can do a run to position using the phoenix stuff
        boolean tempFoundAllPositions = true;
        for (boolean moduleAtPosition : reachedPositionForModule) {
            if (!moduleAtPosition) {
                tempFoundAllPositions = false;
                break;
            }
        }
 
        if (tempFoundAllPositions) {
            foundAllPositions = true;
        }

        return foundAllPositions;
    }

    private boolean atLeastOneAtFinalPosition() {
        for (boolean moduleAtPosition : reachedPositionForModule) {
            if (moduleAtPosition) {
                return true;
            }
        }

        return false;
    }

    private static double deriveMaxTimeoutFromDistance(double distance, DistanceUnit units) {
        double maxTimeout = Math.abs(units.toBaseUnits(distance) / WORST_CASE_METERS_PER_SECOND);
        SmartDashboard.putNumber("maxTimeout", maxTimeout);
        return maxTimeout;
    }
}
