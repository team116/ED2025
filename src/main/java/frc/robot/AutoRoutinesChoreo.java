package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.autos.primitives.SendElevatorToPositionCommand;
import frc.robot.autos.primitives.SendWristToAbsoluteEncoderAngle;
import frc.robot.autos.primitives.ExpelGamePieceCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;

public class AutoRoutinesChoreo {
    private final AutoFactory autoFactory;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Intake intake;

    public AutoRoutinesChoreo(AutoFactory factory, Elevator elevator, Wrist wrist, Intake intake) {
        autoFactory = factory;
        this.elevator = elevator;
        this.wrist = wrist;
        this.intake = intake;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = autoFactory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("SimplePath");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }

    public AutoRoutine blueEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("My Awesome Blue Routine");
        final AutoTrajectory blueStraightTraj = routine.trajectory("BlueStraight");

        routine.active().onTrue(
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("event", "Start blue straight")),
                blueStraightTraj.resetOdometry(),
                blueStraightTraj.cmd()
            )      
        );

        blueStraightTraj.atTime("ElevatorUp").onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "ElevatorUp")));
    
        blueStraightTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "Done blue straight")));

        return routine;
    }

    public AutoRoutine blueLeftEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Bound Blue Alliance");
        final AutoTrajectory blueLeftTraj = routine.trajectory("BlueSimpleLeft");

        routine.active().onTrue(
            Commands.sequence(
                blueLeftTraj.resetOdometry(), 
                blueLeftTraj.cmd()
            )
        );

        blueLeftTraj.atTime("Extend").onTrue(new SendElevatorToPositionCommand(elevator,1.5d,Elevator.LEVEL_1_POSITION)); // On Extend trigger, extend to L1 Position for 1.5 seconds

        blueLeftTraj.atTime("Rotate").onTrue(new SendWristToAbsoluteEncoderAngle(wrist,1.0d,Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE)); // On Rotate trigger, rotate to L1 Position for 1.0 seconds

        blueLeftTraj.atTime("Expel").onTrue(new ExpelGamePieceCommand(intake,2.0d)); // On Expel trigger, Expel for 1.0 seconds

        return routine;
    }

    public AutoRoutine pickupAndScoreAuto() {
        // AutoRoutine routine = autoFactory.newRoutine("pickupAndScore");

        // // Load the routine's trajectories
        // AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
        // AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");

        // // When the routine begins, reset odometry and start the first trajectory 
        // routine.active().onTrue(
        //     Commands.sequence(
        //         pickupTraj.resetOdometry(),
        //         pickupTraj.cmd()
        //     )
        // );

        // // Starting at the event marker named "intake", run the intake 
        // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());

        // // When the trajectory is done, start the next trajectory
        // pickupTraj.done().onTrue(scoreTraj.cmd());

        // // While the trajectory is active, prepare the scoring subsystem
        // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

        // // When the trajectory is done, score
        // scoreTraj.done().onTrue(scoringSubsystem.score());

        // return routine;
        return null;
    }
}