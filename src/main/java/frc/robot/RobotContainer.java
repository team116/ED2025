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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.autos.primitives.MonitorDistanceDriven;
import frc.robot.autos.primitives.SendElevatorToPositionCommand;
import frc.robot.autos.primitives.SendWristToRelativeEncoderAngle;
import frc.robot.commands.AprilTagAutoAlign;
import frc.robot.commands.DefaultClimberCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultWristCommand;
import frc.robot.autos.BruteForceScoreAuto;
import frc.robot.autos.DriveOffTheLine;
import frc.robot.autos.ScoreTroughCenter;
import frc.robot.autos.ScoreTroughCenterNoElevator;
import frc.robot.autos.primitives.DriveDirection;
import frc.robot.autos.primitives.DriveDistance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrainChoreo;
import frc.robot.subsystems.CommandSwerveDrivetrainPathPlanner;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    private static final boolean USE_MANUAL_AUTO_ROUTINES = false; //toggle between using the primitive based autos and the path planning autos
    private static final boolean GUNNER_CONTROLS_CONNECTED = true; //leave true, assume there is a driver station
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
    private final Joystick gunnerPad;
    private final Joystick gunnerLogitech;

    private final JoystickButton wristUpButton;
    private final JoystickButton wristDownButton;
    private final JoystickButton wristResetEncoderButton;
    private final JoystickButton elevatorResetEncoderButton;
    private final JoystickButton climberUpButton;
    private final JoystickButton climberDownButton;
    private final JoystickButton intakeContinualConsumeButton;
    private final JoystickButton intakeContinualExpelButton;
    private final JoystickButton intakeOffButton;
    private final JoystickButton wristBargeScoreAngleButton;
    private final JoystickButton wristStraightOutAngleButton;
    private final JoystickButton wristGroundPickupAngleButton;
    private final JoystickButton wristProcessorAngleButton;
    private final JoystickButton cancelAllMacrosButton;
    private final JoystickButton elevatorToLowerAlgaeButton;  // Trough position
    //private final JoystickButton elevatorNudgeDownButton;
    private final JoystickButton elevatorToUpperAlgaeButton;
    //private final JoystickButton elevatorNudgeUpButton;
    //private final JoystickButton elevatorToCoralStationIntakeButton;
    private final JoystickButton elevatorToNetButton;
    private final JoystickButton elevatorToBottomButton;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Climber climber = new Climber();
    public final Intake intake = new Intake();
    public final Wrist wrist = new Wrist();

    private static final double WRIST_TIMEOUT = 1.5d;

    public final Command defaultElevatorCommand;
    public final Command defaultWristCommand;
    public final Command defaultIntakeCommand;
    public final Command defaultClimberCommand;

    private final Command sendWristToBargeScoreAngle;// = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_CORAL_STATION_INTAKE_ANGLE);
    private final Command sendWristToStraightOutAngle;// = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE);
    private final Command sendWristToGroundPickupAngle;// = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_DOWN_FULL_ANGLE);
    private final Command sendWristToProcessorAngle;// = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_DOWN_FULL_ANGLE);

    private static final double ELEVATOR_TIMEOUT = 2.0d;

    private final Command sendElevatorToLowerAlgae;
    //private final Command sendElevatorToLevel2;
    private final Command sendElevatorToUpperAlgae;
    //private final Command sendElevatorToLevel4;
    //private final Command sendElevatorToCoralStationIntake;
    private final Command sendElevatorToNet;
    private final Command sendElevatorToBottom;

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
    public AutoRoutinesChoreo autoRoutinesChoreo;

    public RobotContainer() {
        if (drivetrain instanceof CommandSwerveDrivetrainChoreo drivetrainChoreo) {
            autoChooserChoreo = new AutoChooser();
            AutoFactory autoFactoryChoreo = drivetrainChoreo.createAutoFactory();
            autoRoutinesChoreo = new AutoRoutinesChoreo(autoFactoryChoreo, elevator, wrist, intake);

            //autoChooserChoreo.addRoutine("SimplePath", autoRoutinesChoreo::simplePathAuto);
            //autoChooserChoreo.addRoutine("Blue Center", autoRoutinesChoreo::blueStraightEasy);
            //autoChooserChoreo.addRoutine("Blue Left", autoRoutinesChoreo::blueLeftEasy);
            //autoChooserChoreo.addRoutine("Blue Right", autoRoutinesChoreo::blueRightEasy);
            //autoChooserChoreo.addRoutine("Red Center", autoRoutinesChoreo::redStraightEasy);
            //autoChooserChoreo.addRoutine("Red Left", autoRoutinesChoreo::redLeftEasy);
            //autoChooserChoreo.addRoutine("Red Right", autoRoutinesChoreo::redRightEasy);
            //autoChooserChoreo.addRoutine("Blue Left Algae", autoRoutinesChoreo::blueLeftAlgae);
            //autoChooserChoreo.addRoutine("Red Left Algae", autoRoutinesChoreo::redLeftAlgae);
            //autoChooserChoreo.addRoutine("Blue Center Algae", autoRoutinesChoreo::blueCenterProcessorDontUse);
            autoChooserChoreo.addRoutine("Blue Center Processor", autoRoutinesChoreo::blueCenterProcessor);
            autoChooserChoreo.addRoutine("Blue Center Barge", autoRoutinesChoreo::blueCenterBarge);
            autoChooserChoreo.addRoutine("Blue Barge Algae", autoRoutinesChoreo::blueBargeAlgae);
            //autoChooserChoreo.addRoutine("NewPath", autoRoutinesChoreo::NewPath);

            SmartDashboard.putData("Auto Choreo", autoChooserChoreo);
        }

        if (drivetrain instanceof CommandSwerveDrivetrainPathPlanner) {
            autoChooserPathPlanner = AutoBuilder.buildAutoChooser("Tests");
            SmartDashboard.putData("Auto PathPlanner", autoChooserPathPlanner);
        }

        if (USE_MANUAL_AUTO_ROUTINES) {
            autoManual = new SendableChooser<>();
            autoManual.addOption("Do Nothing", new InstantCommand());
            autoManual.addOption("Drive Off The Line", new DriveOffTheLine(drivetrain));
            autoManual.addOption("Brute Force Auto", new BruteForceScoreAuto(drivetrain, elevator, intake, wrist));
            autoManual.addOption("Score Center Trough", new ScoreTroughCenter(drivetrain, elevator, intake, wrist));
            autoManual.addOption("Score Center Trough No Elevator", new ScoreTroughCenterNoElevator(drivetrain, intake, wrist));
            SmartDashboard.putData("Auto Manual", autoManual);
        }

        SmartDashboard.putNumber("MaxSpeed", MaxSpeed);

        configureBindings();

        if (GUNNER_CONTROLS_CONNECTED) {
            gunnerPad = new Joystick(1);
            gunnerLogitech = new Joystick(2);
            
            //gunner stick
            //wrist
            wristUpButton = new JoystickButton(gunnerLogitech, 9);
            wristDownButton = new JoystickButton(gunnerLogitech, 10);
            wristBargeScoreAngleButton = new JoystickButton(gunnerLogitech, 5);
            wristStraightOutAngleButton = new JoystickButton(gunnerLogitech,6);
            wristGroundPickupAngleButton = new JoystickButton(gunnerLogitech, 8);
            wristProcessorAngleButton = new JoystickButton(gunnerLogitech, 7);
            wristResetEncoderButton = new JoystickButton(gunnerLogitech, 11);
            //intake
            intakeContinualConsumeButton = new JoystickButton(gunnerLogitech, 1);
            intakeContinualExpelButton = new JoystickButton(gunnerLogitech, 2);
            intakeOffButton = new JoystickButton(gunnerLogitech, 4);
            
            cancelAllMacrosButton = new JoystickButton(gunnerLogitech, 3);

            //elevator
            elevatorResetEncoderButton = new JoystickButton(gunnerLogitech, 12);
            //gunner pad
            elevatorToLowerAlgaeButton = new JoystickButton(gunnerPad, 8);
            //elevatorNudgeDownButton = new JoystickButton(gunnerPad, 6);
            elevatorToUpperAlgaeButton = new JoystickButton(gunnerPad, 7);
            //elevatorNudgeUpButton = new JoystickButton(gunnerPad, 5);
            //elevatorToCoralStationIntakeButton = new JoystickButton(gunnerPad, 2);
            elevatorToNetButton = new JoystickButton(gunnerPad, 1);
            elevatorToBottomButton = new JoystickButton(gunnerPad, 11);
            //climber
            climberUpButton = new JoystickButton(gunnerPad, 14); // spike up, the actual mechanism moves up, not the robot
            climberDownButton = new JoystickButton(gunnerPad, 10); //spike down, the actual mechanism moves down

            defaultElevatorCommand = new DefaultElevatorCommand(elevator, gunnerLogitech, gunnerPad);
            defaultWristCommand = new DefaultWristCommand(wrist, gunnerLogitech);
            defaultIntakeCommand = new DefaultIntakeCommand(intake, gunnerLogitech, gunnerPad);
            defaultClimberCommand = new DefaultClimberCommand(climber, gunnerPad);

            if (defaultWristCommand instanceof DesiredAngleCallback desiredAngleCallback) {
                sendWristToBargeScoreAngle = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_BARGE_SCORE_ANGLE, desiredAngleCallback);
                sendWristToStraightOutAngle = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_STRAIGHT_OUT_ANGLE, desiredAngleCallback);
                sendWristToGroundPickupAngle = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_GROUND_PICKUP_ANGLE, desiredAngleCallback);
                sendWristToProcessorAngle = new SendWristToRelativeEncoderAngle(wrist, WRIST_TIMEOUT, Wrist.WRIST_PROCESSOR_SCORE_ANGLE, desiredAngleCallback);
            } else {
                sendWristToBargeScoreAngle = new InstantCommand();
                sendWristToStraightOutAngle = new InstantCommand();
                sendWristToGroundPickupAngle = new InstantCommand();
                sendWristToProcessorAngle = new InstantCommand();
            }

            if (defaultElevatorCommand instanceof StallOnInit stallOnInitCallback) {
                sendElevatorToLowerAlgae = new SendElevatorToPositionCommand(elevator, ELEVATOR_TIMEOUT, Elevator.LOWER_ALGAE_POSITION, stallOnInitCallback);
                //sendElevatorToLevel2 = new SendElevatorToPositionCommand(elevator, ELEVATOR_TIMEOUT, Elevator.LEVEL_2_ALGAE_DISLODGE_POSITION, stallOnInitCallback);
                sendElevatorToUpperAlgae = new SendElevatorToPositionCommand(elevator, ELEVATOR_TIMEOUT, Elevator.UPPER_ALGAE_POSITION, stallOnInitCallback);
                //sendElevatorToLevel4 = new SendElevatorToPositionCommand(elevator, ELEVATOR_TIMEOUT, Elevator.LEVEL_4_POSITION, stallOnInitCallback);
                //sendElevatorToCoralStationIntake = new SendElevatorToPositionCommand(elevator, ELEVATOR_TIMEOUT, Elevator.CORAL_STATION_INTAKE_POSITION, stallOnInitCallback);
                sendElevatorToNet = new SendElevatorToPositionCommand(elevator, ELEVATOR_TIMEOUT, Elevator.NET_POSITION, stallOnInitCallback);
                sendElevatorToBottom = new SendElevatorToPositionCommand(elevator, ELEVATOR_TIMEOUT, Elevator.BOTTOM_POSITION, stallOnInitCallback);
            } else {
                sendElevatorToLowerAlgae = new InstantCommand();
                //sendElevatorToLevel2 = new InstantCommand();
                sendElevatorToUpperAlgae = new InstantCommand();
                //sendElevatorToLevel4 = new InstantCommand();
                //sendElevatorToCoralStationIntake = new InstantCommand();
                sendElevatorToNet = new InstantCommand();
                sendElevatorToBottom = new InstantCommand();
            }
            configureGunnerBindings();
        } else {
            gunnerPad = null;
            gunnerLogitech = null;
            wristUpButton = null;
            wristDownButton = null;
            wristResetEncoderButton = null;
            elevatorResetEncoderButton = null;
            climberUpButton = null;
            climberDownButton = null;
            intakeContinualConsumeButton = null;
            intakeContinualExpelButton = null;
            intakeOffButton = null;
            wristBargeScoreAngleButton = null;
            wristStraightOutAngleButton = null;
            wristGroundPickupAngleButton = null;
            wristProcessorAngleButton = null;
            cancelAllMacrosButton = null;
            elevatorToLowerAlgaeButton = null;
            //elevatorNudgeDownButton = null;
            elevatorToUpperAlgaeButton = null;
            //elevatorNudgeUpButton = null;
            //elevatorToCoralStationIntakeButton = null;
            elevatorToNetButton = null;
            elevatorToBottomButton = null;
            defaultElevatorCommand = new InstantCommand();
            defaultWristCommand = new InstantCommand();
            defaultIntakeCommand = new InstantCommand();
            defaultClimberCommand = new InstantCommand();
            sendWristToBargeScoreAngle = new InstantCommand();
            sendWristToStraightOutAngle = new InstantCommand();
            sendWristToGroundPickupAngle = new InstantCommand();
            sendWristToProcessorAngle = new InstantCommand();
            sendElevatorToLowerAlgae = new InstantCommand();
            //sendElevatorToLevel2 = new InstantCommand();
            sendElevatorToUpperAlgae = new InstantCommand();
            //sendElevatorToLevel4 = new InstantCommand();
            //sendElevatorToCoralStationIntake = new InstantCommand();
            sendElevatorToNet = new InstantCommand();
            sendElevatorToBottom = new InstantCommand();
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
        // driverController.pov(90).onTrue(new DriveDistance(drivetrain, DriveDirection.FORWARD, 10, Units.Inches));
        // driverController.pov(270).onTrue(new DriveDistance(drivetrain, DriveDirection.REVERSE, 10, Units.Inches));
        AprilTagAutoAlign alignToRightTag = new AprilTagAutoAlign(drivetrain, true);
        AprilTagAutoAlign alignToLeftTag = new AprilTagAutoAlign(drivetrain, false);
        driverController.pov(90).onTrue(alignToRightTag);
        driverController.pov(270).onTrue(alignToLeftTag);
        driverController.rightTrigger().and(driverController.leftTrigger())
            .onTrue(Commands.runOnce(() -> {
                alignToLeftTag.cancel(); 
                alignToRightTag.cancel();
            }));

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
        elevator.setDefaultCommand(defaultElevatorCommand);
        wrist.setDefaultCommand(defaultWristCommand);
        intake.setDefaultCommand(defaultIntakeCommand);
        climber.setDefaultCommand(defaultClimberCommand);

        //wristUpButton.whileTrue(Commands.startEnd(() -> wrist.up(), () -> wrist.stop()));
        //wristDownButton.whileTrue(Commands.startEnd(() -> wrist.down(), () -> wrist.stop()));

        //wristUpButton.whileTrue(Commands.run(() -> wrist.up()));
        //wristDownButton.whileTrue(Commands.run(() -> wrist.down()));

        //wristUpButton.whileTrue(wrist.startEnd(() -> wrist.up(), () -> wrist.stop()));
        //wristDownButton.whileTrue(wrist.startEnd(() -> wrist.down(), () -> wrist.stop()));
        //climberUpButton.whileTrue(climber.startEnd(() -> climber.pullUpSlow(), () -> climber.stop()));
        //climberDownButton.whileTrue(climber.startEnd(() -> climber.dropDownSlow(), () -> climber.stop()));
        wristResetEncoderButton.onTrue(wrist.runOnce(() -> wrist.resetRelativeEncoder()));
        elevatorResetEncoderButton.onTrue(elevator.runOnce(() -> elevator.resetEncoderPosition()));
        //intakeContinualConsumeButton.whileTrue(intake.runOnce(() -> intake.consume()));
        //intakeContinualExpelButton.whileTrue(intake.runOnce(() -> intake.expel()));

        //intakeContinualConsumeButton.whileTrue(intake.startEnd(() -> intake.consume(), () -> intake.stop()));
        //intakeContinualExpelButton.whileTrue(intake.startEnd(() -> intake.expel(), () -> intake.stop()));

        //intakeContinualConsumeButton.whileTrue(Commands.run(() -> intake.consume()));
        //intakeContinualExpelButton.whileTrue(Commands.run(() -> intake.expel()));

        //intakeOffButton.onTrue(intake.runOnce(() -> intake.stop()));

        // FIXME: Put these back in once we get stuff "working"

        wristBargeScoreAngleButton.onTrue(sendWristToBargeScoreAngle);
        wristStraightOutAngleButton.onTrue(sendWristToStraightOutAngle);
        wristGroundPickupAngleButton.onTrue(sendWristToGroundPickupAngle);
        wristProcessorAngleButton.onTrue(sendWristToProcessorAngle);

        elevatorToBottomButton.onTrue(sendElevatorToBottom);
        //elevatorToCoralStationIntakeButton.onTrue(sendElevatorToCoralStationIntake);
        elevatorToLowerAlgaeButton.onTrue(sendElevatorToLowerAlgae);
        //elevatorNudgeDownButton.onTrue(sendElevatorToLevel2);
        elevatorToUpperAlgaeButton.onTrue(sendElevatorToUpperAlgae);
        //elevatorToLevel4Button.onTrue(sendElevatorToLevel4);
        elevatorToNetButton.onTrue(sendElevatorToNet);

        cancelAllMacrosButton
            .onTrue(Commands.runOnce(() -> sendWristToBargeScoreAngle.cancel()))
            .onTrue(Commands.runOnce(() -> sendWristToStraightOutAngle.cancel()))
            .onTrue(Commands.runOnce(() -> sendWristToGroundPickupAngle.cancel()))
            .onTrue(Commands.runOnce(() -> sendWristToProcessorAngle.cancel()))
            .onTrue(Commands.runOnce(() -> sendElevatorToBottom.cancel()))
            //.onTrue(Commands.runOnce(() -> sendElevatorToCoralStationIntake.cancel()))
            .onTrue(Commands.runOnce(() -> sendElevatorToLowerAlgae.cancel()))
            //.onTrue(Commands.runOnce(() -> sendElevatorToLevel2.cancel()))
            .onTrue(Commands.runOnce(() -> sendElevatorToUpperAlgae.cancel()))
            //.onTrue(Commands.runOnce(() -> sendElevatorToLevel4.cancel()))
            .onTrue(Commands.runOnce(() -> sendElevatorToNet.cancel()));
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
}