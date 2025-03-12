package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ParallelEventOutputBuilder extends Command{

    public static Command parallelPutEvent(String eventName, Command... commands) { // Basically just build a ParallelCommandGroup with the given commands
        Command[] commandList;                                                      // And then it adds in an InstantCommand of putting the event on the SmartDashboard
        commandList = new Command[commands.length + 1];
        for (Integer i = 0; i < commandList.length - 1; i++) {
            commandList[i] = commands[i];
        }
        commandList[commandList.length] = new InstantCommand(() -> SmartDashboard.putString("event",eventName));
    
        return new ParallelCommandGroup(commandList);
    }

}
