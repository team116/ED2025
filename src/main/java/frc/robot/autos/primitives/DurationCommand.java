package frc.robot.autos.primitives;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DurationCommand extends Command {

    private double startTime;
    private final double totalDuration;

    /**
     * Command that naturally finishes once a certain duration has elapsed.
     * 
     * @param totalDuration a non-negative value to represent seconds to execute.
     */
    public DurationCommand(double totalDuration) {
        this.totalDuration = totalDuration;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > totalDuration;
    }

}
