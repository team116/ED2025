package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.primitives.SendElevatorToPositionCommand;
import frc.robot.autos.primitives.SendWristToAbsoluteEncoderAngle;
import frc.robot.autos.primitives.SendWristToRelativeEncoderAngle;
import frc.robot.commands.ParallelEventOutputBuilder;
import frc.robot.autos.primitives.ConsumeGamePieceCommand;
import frc.robot.autos.primitives.DurationCommand;
import frc.robot.autos.primitives.ExpelGamePieceCommand;
import frc.robot.autos.primitives.HoldElevatorAtPosition;
import frc.robot.autos.primitives.HoldWristAtRelativeAngle;
import frc.robot.autos.primitives.MoveWrist;
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

    public AutoRoutine NewPath()
    {
        final AutoRoutine routine = autoFactory.newRoutine("New Path");
        final AutoTrajectory newTraj = routine.trajectory("NewPath");

        routine.active().onTrue(
            newTraj.resetOdometry()
                .andThen(newTraj.cmd())
        );
        return routine;
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

        blueStraightTraj.atTime("Expel").onTrue(
            ParallelEventOutputBuilder.parallelPutEvent(
                "Dump coral",
                Commands.sequence(
                    new MoveWrist(wrist, 1.0, true),
                    new ExpelGamePieceCommand(intake, 0.5),
                    new MoveWrist(wrist, 1.0, false)
                )
            )
        );

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

        //redStraightTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending red straight", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        //redStraightTraj.atTime("Rotate").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Rotating red straight", new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_LEVEL_4_NEUTRAL_ANGLE)));

        //redStraightTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling red straight", new ExpelGamePieceCommand(intake, 2.0d)));
    
        redStraightTraj.done().onTrue(ParallelEventOutputBuilder.parallelPutEvent("Done red straight"));

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

        blueLeftTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending blue left", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        blueLeftTraj.atTime("Rotate").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Rotating blue left", new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_STRAIGHT_OUT_ANGLE))); 
        
        blueLeftTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling blue left", new ExpelGamePieceCommand(intake,2.0d)));
        
        blueLeftTraj.done().onTrue(ParallelEventOutputBuilder.parallelPutEvent("Done blue left"));

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

        redLeftTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending red left", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        redLeftTraj.atTime("Rotate").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Rotating red left", new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_STRAIGHT_OUT_ANGLE))); 
        

        redLeftTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling red left", new ExpelGamePieceCommand(intake,2.0d)));
        
        redLeftTraj.done().onTrue(ParallelEventOutputBuilder.parallelPutEvent("Done red left"));

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

        blueRightTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending blue right", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        blueRightTraj.atTime("Rotate").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Rotating blue right", new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_STRAIGHT_OUT_ANGLE)));

        blueRightTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling blue right", new ExpelGamePieceCommand(intake,2.0d)));

        blueRightTraj.done().onTrue(ParallelEventOutputBuilder.parallelPutEvent("Done blue right"));

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

        redRightTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending red right", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        redRightTraj.atTime("Rotate").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Rotating red right", new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_STRAIGHT_OUT_ANGLE)));

        redRightTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling red right", new ExpelGamePieceCommand(intake,2.0d)));

        redRightTraj.done().onTrue(ParallelEventOutputBuilder.parallelPutEvent("Done red right"));

        return routine;
    }

    public AutoRoutine blueLeftAlgae() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Bound Blue Alliance Algae");
        final AutoTrajectory startBlueLeftAlgaeTraj = routine.trajectory("BlueLeftAlgae");
        final AutoTrajectory consumeBlueLeftAlgaeTraj = routine.trajectory("BlueLeftAlgaeConsume");
        final AutoTrajectory placeBlueLeftAlgaeTraj = routine.trajectory("BlueLeftAlgaePlace");

        routine.active().onTrue(
            Commands.sequence(
                startBlueLeftAlgaeTraj.resetOdometry(),
                startBlueLeftAlgaeTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event","Starting blue left algae")))
            );
        
        startBlueLeftAlgaeTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending blue left algae", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        startBlueLeftAlgaeTraj.atTime("Rotate").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Rotating blue left algae", new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_STRAIGHT_OUT_ANGLE)));

        startBlueLeftAlgaeTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling blue left algae", new ExpelGamePieceCommand(intake, 2.0d)));

        startBlueLeftAlgaeTraj.done().onTrue(consumeBlueLeftAlgaeTraj.cmd());

        consumeBlueLeftAlgaeTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending and intaking blue left algae", Commands.sequence(
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.UPPER_ALGAE_POSITION),
            new ConsumeGamePieceCommand(intake, 2.0d)
        )));

        consumeBlueLeftAlgaeTraj.done().onTrue(placeBlueLeftAlgaeTraj.cmd());

        placeBlueLeftAlgaeTraj.atTime("Retract").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Retracting blue left algae", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.BOTTOM_POSITION)));

        placeBlueLeftAlgaeTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling blue left algae", new ExpelGamePieceCommand(intake, 2.0d)));

        placeBlueLeftAlgaeTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event","Done blue left algae")));

        return routine;
    }

    public AutoRoutine redLeftAlgae() {
        final AutoRoutine routine = autoFactory.newRoutine("Left Bound Red Alliance Algae");
        final AutoTrajectory startRedLeftAlgaeTraj = routine.trajectory("RedLeftAlgae");
        final AutoTrajectory consumeRedLeftAlgaeTraj = routine.trajectory("RedLeftAlgaeConsume");
        final AutoTrajectory placeRedLeftAlgaeTraj = routine.trajectory("RedLeftAlgaePlace");

        routine.active().onTrue(
            Commands.sequence(
                startRedLeftAlgaeTraj.resetOdometry(),
                startRedLeftAlgaeTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event","Starting red left algae")))
            );
        
        startRedLeftAlgaeTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending red left algae", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.LEVEL_1_POSITION)));

        startRedLeftAlgaeTraj.atTime("Rotate").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Rotating red left algae", new SendWristToAbsoluteEncoderAngle(wrist, 1.0d, Wrist.WRIST_STRAIGHT_OUT_ANGLE)));

        startRedLeftAlgaeTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling red left algae", new ExpelGamePieceCommand(intake, 2.0d)));

        startRedLeftAlgaeTraj.done().onTrue(consumeRedLeftAlgaeTraj.cmd());

        consumeRedLeftAlgaeTraj.atTime("Extend").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Extending and intaking red left algae", Commands.sequence(
            new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.UPPER_ALGAE_POSITION),
            new ConsumeGamePieceCommand(intake, 2.0d)
        )));

        consumeRedLeftAlgaeTraj.done().onTrue(placeRedLeftAlgaeTraj.cmd());

        placeRedLeftAlgaeTraj.atTime("Retract").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Retracting red left algae", new SendElevatorToPositionCommand(elevator, 1.5d, Elevator.BOTTOM_POSITION)));

        placeRedLeftAlgaeTraj.atTime("Expel").onTrue(ParallelEventOutputBuilder.parallelPutEvent("Expelling red left algae", new ExpelGamePieceCommand(intake, 2.0d)));

        placeRedLeftAlgaeTraj.done().onTrue(new InstantCommand(() -> SmartDashboard.putString("event","Done red left algae")));

        return routine;
    }

    private AtomicBoolean isDone1 = new AtomicBoolean(false);
    private Trigger done1 = new Trigger(() -> isDone1.get());
    private AtomicBoolean isDone2 = new AtomicBoolean(false);
    private Trigger done2 = new Trigger(() -> isDone2.get());
    private AtomicBoolean isDone3 = new AtomicBoolean(false);
    private Trigger done3 = new Trigger(() -> isDone3.get());
    private AtomicBoolean isDone4 = new AtomicBoolean(false);
    private Trigger done4 = new Trigger(() -> isDone4.get());

    public void clearTriggers() {
        isDone1.set(false);
        isDone2.set(false);
        isDone3.set(false);
    }

    public AutoRoutine blueProcessorAlgae() { //FIXME: Still needs to be tested
        AutoRoutine routine = autoFactory.newRoutine("Blue Processor Bound Algae");

        AutoTrajectory blueProcessorTraj1 = routine.trajectory("ProcessorAlgae1");
        AutoTrajectory blueProcessorTraj2 = routine.trajectory("ProcessorAlgae2");
        AutoTrajectory blueProcessorTraj3 = routine.trajectory("ProcessorAlgae3");

        routine.active().onTrue(
            blueProcessorTraj1.resetOdometry().andThen(
                blueProcessorTraj1.cmd(),
                new InstantCommand(() -> wrist.stall())
            )
        );

        blueProcessorTraj1.done().onTrue(
            Commands.sequence(
            new MoveWrist(wrist, 0.8, true),
            new ExpelGamePieceCommand(intake, 0.3),
            new MoveWrist(wrist, 0.9, false),
            new InstantCommand(() -> wrist.resetRelativeEncoder()), 
            new InstantCommand(() -> isDone1.set(true))
        ));

        routine.observe(done1).onTrue(blueProcessorTraj2.cmd());

        blueProcessorTraj2.done().onTrue(
            Commands.sequence(
                new SendWristToRelativeEncoderAngle(wrist, 3.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new SendElevatorToPositionCommand(elevator, 8.0, Elevator.UPPER_ALGAE_POSITION),
                new InstantCommand(() -> intake.consume()),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new InstantCommand(() -> isDone2.set(true))
            ));

        routine.observe(done2).onTrue(blueProcessorTraj3.cmd());

        Command holdWristStraightOut = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_STRAIGHT_OUT_ANGLE);
        Command holdElevatorAtUpperAlgaePosition = new HoldElevatorAtPosition(elevator, Double.MAX_VALUE, Elevator.UPPER_ALGAE_POSITION);

        blueProcessorTraj3.active().onTrue(
            Commands.parallel(
                holdWristStraightOut,
                holdElevatorAtUpperAlgaePosition
            )
        );

        blueProcessorTraj3.done().onTrue(
            Commands.sequence(
                new InstantCommand(() -> holdWristStraightOut.cancel()),                
                new SendElevatorToPositionCommand(elevator, 4.0, Elevator.PROCESSOR_POSITION),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_PROCESSOR_SCORE_ANGLE),
                new ExpelGamePieceCommand(intake, 1.5),
                new DurationCommand(1.5),
                new InstantCommand(() -> intake.stop())
            )
        );

        return routine;
    }

    public AutoRoutine blueBargeAlgaeSuperLaunch() {
        AutoRoutine routine = autoFactory.newRoutine("blue barge algae super launch");

        AutoTrajectory blueBargeTraj1 = routine.trajectory("BargeAlgae1");
        AutoTrajectory blueBargeTraj2 = routine.trajectory("BargeAlgae2");
        AutoTrajectory blueBargeTraj3 = routine.trajectory("BargeAlgaeSuperLaunch");
        //AutoTrajectory blueBargeTraj4 = routine.trajectory("BlueBargeAlgae4");

        routine.active().onTrue(
            blueBargeTraj1.resetOdometry().andThen(
                blueBargeTraj1.cmd(),
                new InstantCommand(() -> wrist.stall())
            )
        );

        blueBargeTraj1.done().onTrue(
            Commands.sequence(
            new MoveWrist(wrist, 0.8, true),
            new ExpelGamePieceCommand(intake, 0.3),
            new MoveWrist(wrist, 0.9, false),
            new InstantCommand(() -> wrist.resetRelativeEncoder()), 
            new InstantCommand(() -> isDone1.set(true))
        ));

        routine.observe(done1).onTrue(
            blueBargeTraj2.cmd()
        );

        blueBargeTraj2.done().onTrue(
            Commands.sequence(
                new SendWristToRelativeEncoderAngle(wrist, 3.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new SendElevatorToPositionCommand(elevator, 8.0, Elevator.UPPER_ALGAE_POSITION),
                new InstantCommand(() -> intake.consume()),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new InstantCommand(() -> isDone2.set(true))
            )
        );

        routine.observe(done2).onTrue(
            blueBargeTraj3.cmd()
        );

        Command holdWristStraightOut = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_STRAIGHT_OUT_ANGLE);
        Command holdElevatorAtUpperAlgaePosition = new HoldElevatorAtPosition(elevator, Double.MAX_VALUE, Elevator.UPPER_ALGAE_POSITION);

        blueBargeTraj3.active().onTrue(
            Commands.parallel(
                holdWristStraightOut,
                holdElevatorAtUpperAlgaePosition
            )
        );
        // Command moveDownCommands = Commands.parallel( // Commands to move the wrist and elevator back to starting point
        //     new SendWristToRelativeEncoderAngle(wrist,1.5d,110.0d),
        //     new SendElevatorToPositionCommand(elevator, 1.0d, Elevator.BOTTOM_POSITION)
        // );
        blueBargeTraj3.done().onTrue(
            Commands.sequence(
                new InstantCommand(() -> holdElevatorAtUpperAlgaePosition.cancel()),
                new InstantCommand(() -> SmartDashboard.putString("event", "Send Elevator To Net")),
                new SendElevatorToPositionCommand(elevator, 1.75, Elevator.NET_POSITION),
                new InstantCommand(() -> SmartDashboard.putString("event", "Start Cancel Wrist")),
                new InstantCommand(() -> holdWristStraightOut.cancel()),
                new InstantCommand(() -> SmartDashboard.putString("event", "Adjust Wrist Angle")),
                new SendWristToRelativeEncoderAngle(wrist, 2.0, Wrist.WRIST_BARGE_SCORE_ANGLE),
                new InstantCommand(() -> SmartDashboard.putString("event", "About To Launch")),
                new InstantCommand(() -> intake.superLaunch()), //super launch?
                new InstantCommand(() -> isDone3.set(true))
                // moveDownCommands
            )
        );

        // routine.observe(done3).onTrue(
        //     blueBargeTraj4.cmd()
        // );

        // blueBargeTraj4.done().onTrue(
        //   new InstantCommand(() -> intake.stop())  
        // );

        return routine;
    }

    public AutoRoutine blueBargeAlgaeManeuver() {
        AutoRoutine routine = autoFactory.newRoutine("blue barge algae maneuver");

        AutoTrajectory blueBargeTraj1 = routine.trajectory("BargeAlgae1");
        AutoTrajectory blueBargeTraj2 = routine.trajectory("BargeAlgae2");
        AutoTrajectory blueBargeTraj3 = routine.trajectory("BargeAlgaeManeuver");
        

        routine.active().onTrue(
            blueBargeTraj1.resetOdometry().andThen(
                blueBargeTraj1.cmd(),
                new InstantCommand(() -> wrist.stall())
            )
        );

        blueBargeTraj1.done().onTrue(
            Commands.sequence(
            new MoveWrist(wrist, 0.8, true),
            new ExpelGamePieceCommand(intake, 0.3),
            new MoveWrist(wrist, 0.9, false),
            new InstantCommand(() -> wrist.resetRelativeEncoder()), 
            new InstantCommand(() -> isDone1.set(true))
        ));

        routine.observe(done1).onTrue(
            blueBargeTraj2.cmd()
        );

        blueBargeTraj2.done().onTrue(
            Commands.sequence(
                new SendWristToRelativeEncoderAngle(wrist, 3.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new SendElevatorToPositionCommand(elevator, 8.0, Elevator.UPPER_ALGAE_POSITION),
                new InstantCommand(() -> intake.consume()),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new InstantCommand(() -> isDone2.set(true))
            )
        );

        routine.observe(done2).onTrue(
            blueBargeTraj3.cmd()
        );

        Command holdWristStraightOut = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_STRAIGHT_OUT_ANGLE);
        Command holdElevatorAtUpperAlgaePosition = new HoldElevatorAtPosition(elevator, Double.MAX_VALUE, Elevator.UPPER_ALGAE_POSITION);

        blueBargeTraj3.active().onTrue(
            Commands.parallel(
                holdWristStraightOut,
                holdElevatorAtUpperAlgaePosition
            )
        );
        Command moveDownCommands = Commands.parallel(
            new SendWristToRelativeEncoderAngle(wrist,1.5d,110.0d),
            new SendElevatorToPositionCommand(elevator, 1.0d, Elevator.BOTTOM_POSITION)
        );
        blueBargeTraj3.done().onTrue(
            Commands.sequence(
                new InstantCommand(() -> holdElevatorAtUpperAlgaePosition.cancel()),
                new InstantCommand(() -> SmartDashboard.putString("event", "Send Elevator To Net")),
                new SendElevatorToPositionCommand(elevator, 1.75, Elevator.NET_POSITION),
                new InstantCommand(() -> SmartDashboard.putString("event", "Start Cancel Wrist")),
                new InstantCommand(() -> holdWristStraightOut.cancel()),
                new InstantCommand(() -> SmartDashboard.putString("event", "Adjust Wrist Angle")),
                new SendWristToRelativeEncoderAngle(wrist, 3.0, Wrist.WRIST_BARGE_SCORE_ANGLE),
                new InstantCommand(() -> SmartDashboard.putString("event", "About To Launch")),
                new InstantCommand(() -> intake.launch()), //super launch?
                new DurationCommand(.5),
                new InstantCommand(() -> SmartDashboard.putString("event", "Launched")),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new InstantCommand(() -> intake.stop()),
                moveDownCommands
            )
        );

        return routine;
    }

    // Don't use this version, use the blueCenterProcessor that shares code with blueCenterBarge
    public AutoRoutine blueCenterProcessorDontUse() {
        AutoRoutine routine = autoFactory.newRoutine("Center Bound Blue Alliance Algae");
        AutoTrajectory blueStraightTraj = routine.trajectory("StraightAlgae");
        AutoTrajectory blueStraightTraj2 = routine.trajectory("StraightAlgae2");
        AutoTrajectory blueStraightTraj3 = routine.trajectory("StraightAlgae3");
        AutoTrajectory blueStraightTraj4 = routine.trajectory("StraightAlgae4");

        routine.active().onTrue(
            blueStraightTraj.resetOdometry().andThen(
                blueStraightTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event", "Starting blue center algae")),
                new InstantCommand(() -> wrist.stall())
            )
        );

        blueStraightTraj.done().onTrue(
            Commands.sequence(
                new MoveWrist(wrist, 0.8, true),
                new ExpelGamePieceCommand(intake, 0.3),
                new MoveWrist(wrist, 0.9, false),
                new InstantCommand(() -> wrist.resetRelativeEncoder()),  // Yes, want to reset after sending to top position
                new InstantCommand(() -> isDone1.set(true))
            )
        );

        routine.observe(done1).onTrue(blueStraightTraj2.cmd());

        blueStraightTraj2.done().onTrue(
            Commands.sequence(
                new SendWristToRelativeEncoderAngle(wrist, 3.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new SendElevatorToPositionCommand(elevator, 8.0, Elevator.LOWER_ALGAE_POSITION),
                new InstantCommand(() -> intake.consume()),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new InstantCommand(() -> isDone2.set(true))
            )
        );

        routine.observe(done2).onTrue(blueStraightTraj3.cmd());
     
        Command holdWristStraightOut = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_STRAIGHT_OUT_ANGLE);
        Command holdElevatorAtLowerAlgaePosition = new HoldElevatorAtPosition(elevator, Double.MAX_VALUE, Elevator.LOWER_ALGAE_POSITION);
        blueStraightTraj3.active().onTrue(
            Commands.parallel(
                holdWristStraightOut,
                holdElevatorAtLowerAlgaePosition
            )
        );

        blueStraightTraj3.done().onTrue(
            Commands.sequence(
                // Commands.parallel(
                //     new SendWristToRelativeEncoderAngle(wrist, 2.0, Wrist.WRIST_PROCESSOR_SCORE_ANGLE),
                //     new SendElevatorToPositionCommand(elevator, 2.0, Elevator.PROCESSOR_POSITION)
                // ),
                new InstantCommand(() -> holdElevatorAtLowerAlgaePosition.cancel()),
                new InstantCommand(() -> isDone3.set(true))
            )
        );

        routine.observe(done3).onTrue(blueStraightTraj4.cmd());

        // Command holdWristForProcessor = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_PROCESSOR_SCORE_ANGLE);
        // Command holdElevatorAtProcessorPosition = new HoldElevatorAtPosition(elevator, Double.MAX_VALUE, Elevator.PROCESSOR_POSITION);
        // blueStraightTraj4.active().onTrue(
        //     Commands.parallel(
        //         holdWristForProcessor,
        //         holdElevatorAtProcessorPosition
        //     )
        // );

        // blueStraightTraj4.active().onTrue(
        //     elevator.runOnce(() -> elevator.stop()) // Should take effect once holdElevatorAtLowerAlgaePosition.cancel() takes effect
        // );

        blueStraightTraj4.done().onTrue(
            Commands.sequence(
                new InstantCommand(() -> holdWristStraightOut.cancel()),
                //new InstantCommand(() -> holdElevatorAtLowerAlgaePosition.cancel()),
                new SendElevatorToPositionCommand(elevator, 4.0, Elevator.PROCESSOR_POSITION),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_PROCESSOR_SCORE_ANGLE),
                new ExpelGamePieceCommand(intake, 1.5),
                new DurationCommand(1.5),
                new InstantCommand(() -> intake.stop())
            )
        );

        return routine;
    }

    public AutoRoutine blueCenterBargeSuperLaunch() {
        AutoRoutine routine = autoFactory.newRoutine("blue center barge super launch");
        AutoTrajectory blueStraightTrajBarge = routine.trajectory("StraightBargeSuperLaunch");
        //AutoTrajectory blueStraightTrajBarge2 = routine.trajectory("BlueStraightBarge2");
        Command holdWristStraightOut = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_STRAIGHT_OUT_ANGLE);

        blueCenterShared(routine, holdWristStraightOut);

        routine.observe(done3).onTrue(blueStraightTrajBarge.cmd());

        blueStraightTrajBarge.done().onTrue(
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("event", "Send Elevator To Net")),
                new SendElevatorToPositionCommand(elevator, 1.75, Elevator.NET_POSITION),
                new InstantCommand(() -> SmartDashboard.putString("event", "Start Cancel Wrist")),
                new InstantCommand(() -> holdWristStraightOut.cancel()),
                new InstantCommand(() -> SmartDashboard.putString("event", "Adjust Wrist Angle")),
                new SendWristToRelativeEncoderAngle(wrist, 2.0, Wrist.WRIST_AUTO_SHOOT_ANGLE), //maybe use a higher shooting angle, Wrist.AUTO_SHOOTING_ANGLE
                new InstantCommand(() -> SmartDashboard.putString("event", "About To Launch")),
                new InstantCommand(() -> intake.superLaunch())//,
                //new InstantCommand(() -> isDone4.set(true))
            )
        );

        // routine.observe(done4).onTrue(
        //   blueStraightTrajBarge2.cmd()  
        // );

        // blueStraightTrajBarge2.active().onTrue(
        //   new InstantCommand(() -> intake.stop())  
        // );

        return routine;
    }

    public AutoRoutine blueCenterBargeManeuver() {
        AutoRoutine routine = autoFactory.newRoutine("blue center barge maneuver");
        AutoTrajectory blueStraightTrajBarge = routine.trajectory("StraightBargeManeuver");
        Command holdWristStraightOut = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_STRAIGHT_OUT_ANGLE);

        blueCenterShared(routine, holdWristStraightOut);

        routine.observe(done3).onTrue(blueStraightTrajBarge.cmd());

        blueStraightTrajBarge.done().onTrue(
            Commands.sequence(
                new InstantCommand(() -> SmartDashboard.putString("event", "Send Elevator To Net")),
                new SendElevatorToPositionCommand(elevator, 1.75, Elevator.NET_POSITION),
                new InstantCommand(() -> SmartDashboard.putString("event", "Start Cancel Wrist")),
                new InstantCommand(() -> holdWristStraightOut.cancel()),
                new InstantCommand(() -> SmartDashboard.putString("event", "Adjust Wrist Angle")),
                new SendWristToRelativeEncoderAngle(wrist, 3.0, Wrist.WRIST_BARGE_SCORE_ANGLE),
                new InstantCommand(() -> SmartDashboard.putString("event", "About To Launch")),
                new InstantCommand(() -> intake.launch()),
                new DurationCommand(.5),
                new InstantCommand(() -> SmartDashboard.putString("event", "Launched")),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new InstantCommand(() -> intake.stop())
            )
        );

        return routine;
    }

    public AutoRoutine blueCenterProcessor() {
        AutoRoutine routine = autoFactory.newRoutine("blue center processor");
        AutoTrajectory blueStraightTraj4 = routine.trajectory("StraightAlgae4");
        Command holdWristStraightOut = new HoldWristAtRelativeAngle(wrist, Double.MAX_VALUE, Wrist.WRIST_STRAIGHT_OUT_ANGLE);

        blueCenterShared(routine, holdWristStraightOut);

        routine.observe(done3).onTrue(blueStraightTraj4.cmd());

        blueStraightTraj4.done().onTrue(
            Commands.sequence(
                new InstantCommand(() -> holdWristStraightOut.cancel()),
                //new InstantCommand(() -> holdElevatorAtLowerAlgaePosition.cancel()),
                new SendElevatorToPositionCommand(elevator, 4.0, Elevator.PROCESSOR_POSITION),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_PROCESSOR_SCORE_ANGLE),
                new ExpelGamePieceCommand(intake, 1.5),
                new DurationCommand(1.5),
                new InstantCommand(() -> intake.stop())
            )
        );

        return routine;
    }

    private void blueCenterShared(AutoRoutine routine, Command holdWristStraightOut) {
        AutoTrajectory blueStraightTraj = routine.trajectory("StraightAlgae");
        AutoTrajectory blueStraightTraj2 = routine.trajectory("StraightAlgae2");
        AutoTrajectory blueStraightTraj3 = routine.trajectory("StraightAlgae3");

        routine.active().onTrue(
            blueStraightTraj.resetOdometry().andThen(
                blueStraightTraj.cmd(),
                new InstantCommand(() -> SmartDashboard.putString("event", "Starting blue center algae")),
                new InstantCommand(() -> wrist.stall())
            )
        );

        blueStraightTraj.done().onTrue(
            Commands.sequence(
                new MoveWrist(wrist, 0.8, true),
                new ExpelGamePieceCommand(intake, 0.3),
                new MoveWrist(wrist, 0.9, false),
                new InstantCommand(() -> wrist.resetRelativeEncoder()),  // Yes, want to reset after sending to top position
                new InstantCommand(() -> isDone1.set(true))
            )
        );

        routine.observe(done1).onTrue(blueStraightTraj2.cmd());

        blueStraightTraj2.done().onTrue(
            Commands.sequence(
                new SendWristToRelativeEncoderAngle(wrist, 3.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new SendElevatorToPositionCommand(elevator, 8.0, Elevator.LOWER_ALGAE_POSITION),
                new InstantCommand(() -> intake.consume()),
                new SendWristToRelativeEncoderAngle(wrist, 1.0, Wrist.WRIST_STRAIGHT_OUT_ANGLE),
                new InstantCommand(() -> isDone2.set(true))
            )
        );

        routine.observe(done2).onTrue(blueStraightTraj3.cmd());
     
        Command holdElevatorAtLowerAlgaePosition = new HoldElevatorAtPosition(elevator, Double.MAX_VALUE, Elevator.LOWER_ALGAE_POSITION);
        blueStraightTraj3.active().onTrue(
            Commands.parallel(
                holdWristStraightOut,
                holdElevatorAtLowerAlgaePosition
            )
        );

        blueStraightTraj3.done().onTrue(
            Commands.sequence(
                // Commands.parallel(
                //     new SendWristToRelativeEncoderAngle(wrist, 2.0, Wrist.WRIST_PROCESSOR_SCORE_ANGLE),
                //     new SendElevatorToPositionCommand(elevator, 2.0, Elevator.PROCESSOR_POSITION)
                // ),
                new InstantCommand(() -> holdElevatorAtLowerAlgaePosition.cancel()),
                new InstantCommand(() -> isDone3.set(true))
            )
        );
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