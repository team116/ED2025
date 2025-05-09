// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.util.TrajSchemaVersion;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // NOTE: Only enable for debugging as it really slows down things
    // if (LimelightHelpers.getTV(Constants.LIMELIGHT_NAME)) {
    //   double visibleAprilTag = LimelightHelpers.getFiducialID(Constants.LIMELIGHT_NAME);
    //   SmartDashboard.putString(RobotContainer.APRIL_TAG_KEY, Double.toString(visibleAprilTag));
    // } else {
    //   SmartDashboard.putString(RobotContainer.APRIL_TAG_KEY, "no target found");
    // }

    SmartDashboard.putNumber("Elevator Position", m_robotContainer.elevator.getEncoderPosition());
    SmartDashboard.putNumber("Wrist Position", m_robotContainer.wrist.getRelativePosition());
    SmartDashboard.putNumber("Wrist Angle", m_robotContainer.wrist.getRelativeAngle());
    SmartDashboard.putNumber("Climber Position", m_robotContainer.climber. getEncoderPosition());
    SmartDashboard.putBoolean("Coral Limit Switch", m_robotContainer.intake.coralLimitSwitchIsPressed());
    SmartDashboard.putBoolean("Algae Limit Switch", m_robotContainer.intake.algaeLimitSwitchIsPressed());
    SmartDashboard.putString("DriveTrain ", m_robotContainer.drivetrain.getOperatorForwardDirection().toString());
    //m_robotContainer.intake.printMotorVoltages();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainerOriginal} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.wrist.resetRelativeEncoder();
    m_robotContainer.elevator.resetEncoderPosition();

    if (m_robotContainer.autoRoutinesChoreo != null) {
      m_robotContainer.autoRoutinesChoreo.clearTriggers();
    }

    // FIXME: try to put this back
    //m_robotContainer.drivetrain.resetRotation(Rotation2d.k180deg);
    //Pose2d rotatedPose = new Pose2d(Translation2d.kZero, Rotation2d.k180deg);
    //m_robotContainer.drivetrain.resetPose(rotatedPose);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //m_robotContainer.wrist.resetRelativeEncoder();
    //m_robotContainer.elevator.stall();

    SmartDashboard.putString(RobotContainer.SPEED_MODE_KEY, RobotContainer.FAST_MODE);
    // NOTE: The bottom line might have crashed the program
    //Shuffleboard.getTab("SmartDashboard").addCamera("Cameras", "limelight", "http://10.1.16.14:5800");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
