package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRoutinesPathPlanner {

    public AutoRoutinesPathPlanner() {}

    public Command followTopBoundBlueAlliance() {
        return AutoBuilder.buildAuto("TopBoundBlueAlliance");
    }

    public Command followCenterBoundBlueAlliance() {
        return AutoBuilder.buildAuto("CenterBoundBlueAlliance");
    }

    public Command followBottomBoundBlueAlliane() {
        return AutoBuilder.buildAuto("BottomBoundBlueAlliance");
    }

    public Command followTopBoundRedAlliance() {
        return AutoBuilder.buildAuto("TopBoundRedAlliance");
    }

    public Command followCenterBoundRedAlliance() {
        return AutoBuilder.buildAuto("CenterBoundRedAlliance");
    }

    public Command followBottomBoundRedAlliance() {
        return AutoBuilder.buildAuto("BottomBoundRedAlliance");
    }
}