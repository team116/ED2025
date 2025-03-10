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

    public AutoRoutine blueStraightEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("My Awesome Blue Routine");
        final AutoTrajectory blueStraightTraj = routine.trajectory("BlueStraight");

        routine.active().onTrue(
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("event", "Start blue straight")),
                blueStraightTraj.resetOdometry(),
                blueStraightTraj.cmd()
            )      
        );

        blueStraightTraj.atTime("Extend").onTrue(Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putString("event", "Extend blue straight")),
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        blueStraightTraj.atTime("Rotate").onTrue(Commands.parallel(
             new InstantCommand(() -> SmartDashboard.putString("event", "Rotate blue straight")),
             new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE)
        ));

        blueStraightTraj.atTime("Expel").onTrue(Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putString("event", "Expel blue straight")),
            new ExpelGamePieceCommand(intake, 2.0d)
        ));
    
        blueStraightTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "Done blue straight")));

        return routine;
    }

    public AutoRoutine redStraightEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("My Awesome Red Routine");
        final AutoTrajectory redStraightTraj = routine.trajectory("RedStraight");

        routine.active().onTrue(
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("event", "Start red straight")),
                redStraightTraj.resetOdometry(),
                redStraightTraj.cmd()
            )      
        );

        redStraightTraj.atTime("Extend").onTrue(Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putString("event", "Extend red straight")),
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        redStraightTraj.atTime("Rotate").onTrue(Commands.parallel(
             new InstantCommand(() -> SmartDashboard.putString("event", "Rotate red straight")),
             new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE)
        ));

        redStraightTraj.atTime("Expel").onTrue(Commands.parallel(
            new InstantCommand(() -> SmartDashboard.putString("event", "Expel red straight")),
            new ExpelGamePieceCommand(intake, 2.0d)
        ));
    
        redStraightTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "Done red straight")));

        return routine;
    }

    public AutoRoutine blueLeftEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Bound Blue Alliance");
        final AutoTrajectory blueLeftTraj = routine.trajectory("BlueSimpleLeft");

        routine.active().onTrue(
            Commands.sequence(
                blueLeftTraj.resetOdometry(), 
                blueLeftTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event", "Starting blue left"))
            )
        );

        blueLeftTraj.atTime("Extend").onTrue(Commands.parallel(
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION),
            new InstantCommand(() -> SmartDashboard.putString("event", "Extending blue left"))
            )); // On Extend trigger, extend to L1 Position for 1.5 seconds

        blueLeftTraj.atTime("Rotate").onTrue(Commands.parallel(
            new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE),
            new InstantCommand(() -> SmartDashboard.putString("event", "Rotating blue left")))); // On Rotate trigger, rotate to L1 Position for 1.0 seconds

        blueLeftTraj.atTime("Expel").onTrue(Commands.parallel(
            new ExpelGamePieceCommand(intake,2.0d),
            new InstantCommand(() -> SmartDashboard.putString("event", "Expelling blue left")))); // On Expel trigger, Expel for 1.0 seconds
        
        blueLeftTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "Done blue left")));

        return routine;
    }

    public AutoRoutine redLeftEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Bound Red Alliance");
        final AutoTrajectory redLeftTraj = routine.trajectory("RedSimpleLeft");

        routine.active().onTrue(
            Commands.sequence(
                redLeftTraj.resetOdometry(), 
                redLeftTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event", "Starting red left"))
            )
        );

        redLeftTraj.atTime("Extend").onTrue(Commands.parallel(
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION),
            new InstantCommand(() -> SmartDashboard.putString("event", "Extending red left"))
            ));

        redLeftTraj.atTime("Rotate").onTrue(Commands.parallel(
            new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE),
            new InstantCommand(() -> SmartDashboard.putString("event", "Rotating red left")))); 
        
        redLeftTraj.atTime("Expel").onTrue(Commands.parallel(
            new ExpelGamePieceCommand(intake,2.0d),
            new InstantCommand(() -> SmartDashboard.putString("event", "Expelling red left"))));
        
        redLeftTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "Done red left")));

        return routine;
    }

    public AutoRoutine blueRightEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("Right Bound Blue Alliance");
        final AutoTrajectory blueRightTraj = routine.trajectory("BlueSimpleRight");

        routine.active().onTrue(
            Commands.sequence(
                blueRightTraj.resetOdometry(),
                blueRightTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event", "Starting blue right"))
            )
        );

        blueRightTraj.atTime("Extend").onTrue(Commands.parallel(
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION),
            new InstantCommand(() -> SmartDashboard.putString("event", "Extending blue right"))));

        blueRightTraj.atTime("Rotate").onTrue(Commands.parallel(
            new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE),
            new InstantCommand(() -> SmartDashboard.putString("event", "Rotating blue right"))));

        blueRightTraj.atTime("Expel").onTrue(Commands.parallel(
            new ExpelGamePieceCommand(intake,2.0d),
            new InstantCommand(() -> SmartDashboard.putString("event", "Expelling blue right"))));

        blueRightTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "Done blue right")));

        return routine;
    }

    public AutoRoutine redRightEasy() {
        final AutoRoutine routine = autoFactory.newRoutine("Right Bound Red Alliance");
        final AutoTrajectory redRightTraj = routine.trajectory("RedSimpleRight");

        routine.active().onTrue(
            Commands.sequence(
                redRightTraj.resetOdometry(),
                redRightTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event", "Starting red right"))
            )
        );

        redRightTraj.atTime("Extend").onTrue(Commands.parallel(
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION),
            new InstantCommand(() -> SmartDashboard.putString("event", "Extending red right"))));

        redRightTraj.atTime("Rotate").onTrue(Commands.parallel(
            new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE),
            new InstantCommand(() -> SmartDashboard.putString("event", "Rotating red right"))));

        redRightTraj.atTime("Expel").onTrue(Commands.parallel(
            new ExpelGamePieceCommand(intake,2.0d),
            new InstantCommand(() -> SmartDashboard.putString("event", "Expelling red right"))));

        redRightTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event", "Done red right")));

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