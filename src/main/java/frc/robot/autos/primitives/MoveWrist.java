package frc.robot.autos.primitives;

import frc.robot.subsystems.Wrist;

public class MoveWrist extends DurationCommand {

    private Wrist wrist;
    private boolean isDown;

    public MoveWrist(Wrist wrist, double duration, boolean down) {
        super(duration);
        this.wrist = wrist;
        this.isDown = down;
        addRequirements(wrist);
    }


    @Override
    public void execute() {
        super.execute();
        if (isDown) {
            wrist.down();
        } else {
            wrist.up();
        }
    }
}
