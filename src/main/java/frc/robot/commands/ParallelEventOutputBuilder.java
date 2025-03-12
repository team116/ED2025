package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ParallelEventOutputBuilder extends Command{

    public static Command parallelPutEvent(String eventName, Command... commands) { // Basically just build a ParallelCommandGroup with the given commands
        ParallelCommandGroup parallelCommandGroup = new ParallelCommandGroup(commands); // And it includes a new SmartDashboard InstantCommand
        parallelCommandGroup.addCommands(new InstantCommand(() -> SmartDashboard.putString("event",eventName)));
        return parallelCommandGroup;
    }

}
