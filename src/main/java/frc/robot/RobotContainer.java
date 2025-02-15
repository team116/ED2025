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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrainChoreo;
import frc.robot.subsystems.CommandSwerveDrivetrainPathPlanner;

public class RobotContainer {
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

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    //private final AutoFactory autoFactoryChoreo;
    //private final AutoRoutinesChoreo autoRoutinesChoreo;
    private AutoChooser autoChooserChoreo;
    private SendableChooser<Command> autoChooserPathPlanner;

    public RobotContainer() {
        if (drivetrain instanceof CommandSwerveDrivetrainChoreo drivetrainChoreo) {
            autoChooserChoreo = new AutoChooser();
            AutoFactory autoFactoryChoreo = drivetrainChoreo.createAutoFactory();
            AutoRoutinesChoreo autoRoutinesChoreo = new AutoRoutinesChoreo(autoFactoryChoreo);

            autoChooserChoreo.addRoutine("SimplePath", autoRoutinesChoreo::simplePathAuto);
            SmartDashboard.putData("Auto Chooser", autoChooserChoreo);
        }

        if (drivetrain instanceof CommandSwerveDrivetrainPathPlanner) {
            autoChooserPathPlanner = AutoBuilder.buildAutoChooser("Tests");
            SmartDashboard.putData("Auto Mode", autoChooserPathPlanner);
        }

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(shape(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(shape(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(shape(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
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
        if (initial < 0) {
            return -(initial * initial);
        }
        else {
            return initial * initial;
        }
    }
}