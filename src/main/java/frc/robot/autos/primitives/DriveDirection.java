package frc.robot.autos.primitives;

public enum DriveDirection {
    FORWARD(1.0d, 0.0d),
    REVERSE(-1.0d, 0.0d),
    LEFT(0.0d, 1.0d),
    RIGHT(0.0d, -1.0d),
    DIAGONAL_FORWARD_LEFT(1.0d, 1.0d),
    DIAGONAL_FORWARD_RIGHT(1.0d, -1.0d),
    DIAGONAL_BACKWARD_LEFT(-1.0d, 1.0d),
    DIAGONAL_BACKWARD_RIGHT(-1.0d, -1.0d);

    public final double xRatio;
    public final double yRatio;

    private DriveDirection(double xRatio, double yRatio) {
        this.xRatio = xRatio;
        this.yRatio = yRatio;
    }

    public DriveDirection getInverse() {
        switch(this) {
            case FORWARD: return REVERSE;
            case REVERSE: return FORWARD;
            case LEFT:    return RIGHT;
            case RIGHT:   return LEFT;
            case DIAGONAL_FORWARD_LEFT: return DIAGONAL_BACKWARD_RIGHT;
            case DIAGONAL_BACKWARD_RIGHT: return DIAGONAL_FORWARD_LEFT;
            case DIAGONAL_FORWARD_RIGHT: return DIAGONAL_BACKWARD_LEFT;
            case DIAGONAL_BACKWARD_LEFT: return DIAGONAL_FORWARD_RIGHT;
            default: return REVERSE;
        }
    }
}

