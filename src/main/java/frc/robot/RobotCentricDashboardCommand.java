package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotCentricDashboardCommand extends Command {

    private static final String DRIVE_MODE_KEY = "Drive Mode";
    private static final String FIELD_CENTRIC = "Field Centric";
    private static final String ROBOT_CENTRIC = "Robot Centric";

    public RobotCentricDashboardCommand() {
        SmartDashboard.putString(DRIVE_MODE_KEY, FIELD_CENTRIC);

    }

    @Override
    public void initialize() {
        super.initialize();
        SmartDashboard.putString(DRIVE_MODE_KEY, ROBOT_CENTRIC);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString(DRIVE_MODE_KEY, FIELD_CENTRIC);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
