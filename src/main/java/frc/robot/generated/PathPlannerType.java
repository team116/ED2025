package frc.robot.generated;

public enum PathPlannerType {
    CHOREO,
    PATH_PLANNER;

    // NOTE: Change this value to affect which path planning software is used at runtime
    public static final PathPlannerType SELECTED = CHOREO;
}

    // The below code should replace the default createDrivetrain() method in TunerConstants.java
    //
    // /**
    //  * Creates a CommandSwerveDrivetrain instance.
    //  * This should only be called once in your robot program,.
    //  */
    // public static CommandSwerveDrivetrain createDrivetrain() {
    //     if (PathPlannerType.SELECTED == PathPlannerType.CHOREO) {
    //         return new CommandSwerveDrivetrainChoreo(
    //             DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
    //         );
    //     } else {
    //         return new CommandSwerveDrivetrainPathPlanner(
    //             DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
    //         );
    //     }
    // }
