package frc.robot.autos.primitives;

public enum RotationDirection {
    CLOCKWISE(1.0),
    COUNTER_CLOCKWISE(-1.0);

    private final double directionModifier;

    private RotationDirection(double directionModifier) {
        this.directionModifier = directionModifier;
    }

    public double getDirectionModifier() {
        return directionModifier;
    }

    public RotationDirection getInverse() {
        switch(this) {
            case CLOCKWISE: return COUNTER_CLOCKWISE;
            case COUNTER_CLOCKWISE: return CLOCKWISE;
            default: return COUNTER_CLOCKWISE;
        }
    }
}
