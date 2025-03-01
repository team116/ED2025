// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.autos.primitives.MonitorDistanceDriven;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.autos.DriveOffTheLine;
import frc.robot.autos.ScoreTroughCenter;
import frc.robot.autos.primitives.DriveDirection;
import frc.robot.autos.primitives.DriveDistance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrainChoreo;
import frc.robot.subsystems.CommandSwerveDrivetrainPathPlanner;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class RobotContainer {
    private static final boolean USE_MANUAL_AUTO_ROUTINES = false;
    private static final boolean GUNNER_CONTROLS_CONNECTED = true;
    private static final String AUTO_MODE_KEY = "AutoMode";
    private double MaxSpeed = (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) / 2.0d; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = (RotationsPerSecond.of(0.75).in(RadiansPerSecond)) / 2.0d; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final Joystick gunnerStation;
    private final Joystick gunnerLogitech;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Grabber grabber = new Grabber();
    public final Climber climber = new Climber();

    private boolean slowModeActive = false;

    private boolean superSlowModeActive = false;

    public static final String SPEED_MODE_KEY = "Speed";
    public static final String FAST_MODE = "Fast";
    public static final String SLOW_MODE = "Slow";
    public static final String CRAWL_MODE = "Crawl";

    public static final String APRIL_TAG_KEY = "AprilTag in view";

    /* Path follower */
    //private final AutoFactory autoFactoryChoreo;
    //private final AutoRoutinesChoreo autoRoutinesChoreo;
    private AutoChooser autoChooserChoreo;
    private SendableChooser<Command> autoChooserPathPlanner;
    private SendableChooser<Command> autoManual;

    public RobotContainer() {
        if (drivetrain instanceof CommandSwerveDrivetrainChoreo drivetrainChoreo) {
            autoChooserChoreo = new AutoChooser();
            AutoFactory autoFactoryChoreo = drivetrainChoreo.createAutoFactory();
            AutoRoutinesChoreo autoRoutinesChoreo = new AutoRoutinesChoreo(autoFactoryChoreo);

            autoChooserChoreo.addRoutine("SimplePath", autoRoutinesChoreo::simplePathAuto);
            autoChooserChoreo.addRoutine("Blue Center", autoRoutinesChoreo::blueEasy);
            SmartDashboard.putData("Auto Choreo", autoChooserChoreo);
        }

        if (drivetrain instanceof CommandSwerveDrivetrainPathPlanner) {
            autoChooserPathPlanner = AutoBuilder.buildAutoChooser("Tests");
            SmartDashboard.putData("Auto PathPlanner", autoChooserPathPlanner);
        }

        if (USE_MANUAL_AUTO_ROUTINES) {
            autoManual = new SendableChooser<>();
            autoManual.addOption("Drive Off The Line", new DriveOffTheLine(drivetrain));
            autoManual.addOption("Score Center Trough", new ScoreTroughCenter(drivetrain, null, null));
            SmartDashboard.putData("Auto Manual", autoManual);
        }

        SmartDashboard.putNumber("MaxSpeed", MaxSpeed);

        configureBindings();

        if (GUNNER_CONTROLS_CONNECTED) {
            gunnerStation = new Joystick(1);
            gunnerLogitech = new Joystick(2);
            configureGunnerBindings();
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(shape(-driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(shape(-driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(shapeRotation(-driverController.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driverController.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        driverController.pov(90).onTrue(new DriveDistance(drivetrain, DriveDirection.FORWARD, 10, Units.Inches));
        driverController.pov(270).onTrue(new DriveDistance(drivetrain, DriveDirection.REVERSE, 10, Units.Inches));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Toggle robot-centric control
        driverController.rightBumper().toggleOnTrue(drivetrain.applyRequest(() ->
            driveRobotCentric
                .withVelocityX(shape(-driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(shape(-driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(shapeRotation(-driverController.getRightX()) * MaxAngularRate)
            )
        );

        driverController.rightBumper().toggleOnTrue(new RobotCentricDashboardCommand());

        drivetrain.registerTelemetry(logger::telemeterize);

        driverController.x().onTrue(new InstantCommand(() -> toggleSlowMode()));
        driverController.y().onTrue(new InstantCommand(() -> toggleSuperSlowMode()));

        //driverController.rightBumper().onTrue(new InstantCommand(() -> alignToAprilTag()));
    }

    private void configureGunnerBindings() {
        elevator.setDefaultCommand(new DefaultElevatorCommand(elevator, gunnerLogitech));
        // FIXME: Add default commands to grabber (wrist and intake) and climber
    }

    public Command getAutonomousCommand() {
        if (USE_MANUAL_AUTO_ROUTINES) {
            SmartDashboard.putString(AUTO_MODE_KEY, "Manual");
            return autoManual.getSelected();
        }

        if (autoChooserChoreo != null) {
            SmartDashboard.putString(AUTO_MODE_KEY, "Choreo");
            return autoChooserChoreo.selectedCommand();
        } else if (autoChooserPathPlanner != null) {
            SmartDashboard.putString(AUTO_MODE_KEY, "PathPlanner");
            return autoChooserPathPlanner.getSelected();
        } else {
            SmartDashboard.putString(AUTO_MODE_KEY, "Custom");
            return new InstantCommand();
        }
    }

    public double shape(double initial) {
        if(slowModeActive) {
            return (initial * Math.abs(initial)) * 0.75d;
        } else if(superSlowModeActive) {
            return (initial * initial * initial) * 0.50d;
        } else {
           return initial * Math.abs(initial);
        }
    }

    public double shapeRotation(double initial) {
        if(slowModeActive) {
            return (initial * Math.abs(initial));
        } else if(superSlowModeActive) {
            return (initial * initial * initial);
        } else {
           return initial * Math.abs(initial);
        }
    }

    public void toggleSlowMode() {
        slowModeActive = !slowModeActive;
        if (superSlowModeActive) {
            superSlowModeActive = false;
        }
        if (slowModeActive) {
            SmartDashboard.putString(SPEED_MODE_KEY, SLOW_MODE);
        } else {
            SmartDashboard.putString(SPEED_MODE_KEY, FAST_MODE);
        }
    }

    public void toggleSuperSlowMode() {
        superSlowModeActive = !superSlowModeActive;
        if (slowModeActive) {
            slowModeActive = false;
        }
        if (superSlowModeActive) {
            SmartDashboard.putString(SPEED_MODE_KEY, CRAWL_MODE);
        } else {
            SmartDashboard.putString(SPEED_MODE_KEY, FAST_MODE);
        }
    }

    public void alignToAprilTag() {
        double targetAprilTag = LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME);
        //LimelightHelpers.setPriorityTagID("", (int)targetAprilTag);
        //Pose2d position = LimelightHelpers.getBotPoseEstimate(Constants.LIMELIGHT_NAME,"",false).pose;

        //Move with desired position based on april tag ID difference with current position in relation to april tag

        LimelightResults results = LimelightHelpers.getLatestResults(Constants.LIMELIGHT_NAME);

        Pose3d desiredPosition = LimelightHelpers.getTargetPose3d_RobotSpace(null);
        //PoseEstimate desiredSideways = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LIMELIGHT_NAME);
        double distanceToTarget = LimelightHelpers.getBotPoseEstimate_wpiBlue("").rawFiducials[0].distToRobot;

        double offsetSideways = LimelightHelpers.getTX(Constants.LIMELIGHT_NAME);
        double offsetForward = LimelightHelpers.getTY(Constants.LIMELIGHT_NAME);
        LimelightHelpers.getRawFiducials(Constants.LIMELIGHT_NAME);

    }
}