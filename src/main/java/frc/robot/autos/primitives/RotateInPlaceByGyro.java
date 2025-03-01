package frc.robot.autos.primitives;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateInPlaceByGyro extends SequentialCommandGroup {

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric rotateOnly = new SwerveRequest.RobotCentric()
        .withVelocityX(0.0d)
        .withVelocityY(0.0d)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final MonitorAngle monitorAngle;

    public RotateInPlaceByGyro(CommandSwerveDrivetrain drivetrain, double desiredAngle) {
        this.drivetrain = drivetrain;

        monitorAngle = new MonitorAngle(desiredAngle);
        Command rotateInPlace = this.drivetrain.applyRequest(() -> rotateOnly
            .withRotationalRate(getRotationSpeed()));

        addCommands(Commands.deadline(monitorAngle, rotateInPlace));
    }

    private double getRotationSpeed() {
        return monitorAngle.getRotationSpeed();
    }

    private static class MonitorAngle extends DurationCommand {
        private static final int PIGEON_DEVICE_ID = 7 ;
        private static final double DEGREES_AWAY_FROM_DESIRED_THRESHOLD = 0.25;

        private final Pigeon2 gyro = new Pigeon2(PIGEON_DEVICE_ID);
        private final double desiredAngleDegrees;
        private double angleToCheckForInDegrees;
        private int atAngleCount;
        private double rotationSpeed;

        public MonitorAngle(double desiredAngle) {
            super(3.0);  // Maybe figure out by angle

            this.desiredAngleDegrees = desiredAngle;
        }

        public double getRotationSpeed() {
            return rotationSpeed;
        }

        @Override
        public void initialize() {
            super.initialize();
            atAngleCount = 0;
            double currentYaw = gyro.getYaw().getValueAsDouble();
            //System.out.println("currentYaw: " + currentYaw);
            //System.out.println("desiredAngleDegrees: " + desiredAngleDegrees);
            angleToCheckForInDegrees = currentYaw - desiredAngleDegrees;  // Assign to computed angle from current angle
            //System.out.println("angle to check: " + angleToCheckForInDegrees);
            SmartDashboard.putNumber("desiredRotationAngle", angleToCheckForInDegrees);   // FIXME: Remove later
        }
    
        @Override
        public void execute() {
            super.execute();

            SmartDashboard.putNumber("currentRotationAngle", gyro.getYaw().getValueAsDouble());  // FIXME: Remove later
            double angleDifference = angleToCheckForInDegrees - gyro.getYaw().getValueAsDouble();
            double absAngleDifference = Math.abs(angleDifference);
            if (Math.abs(absAngleDifference) < DEGREES_AWAY_FROM_DESIRED_THRESHOLD) {
                ++atAngleCount;
                rotationSpeed = 0.0d;
            } else {
                //System.out.println("currentAngle: " + gyro.getYaw() + " diff: " + angleDifference);
                double absSpeed = 0.2d; //Math.max(Math.min(absAngleDifference / 180.0, maxPercentPower), MIN_PERCENT_POWER);
                rotationSpeed = (angleDifference < 0.0 ? -absSpeed : absSpeed);
                atAngleCount = 0;
            }
        }
    
        @Override
        public void end(boolean interrupted){
            super.end(interrupted);
        }
    
        @Override
        public boolean isFinished() {
            return (atAngleCount > 3 || super.isFinished());
        }
    }
}
