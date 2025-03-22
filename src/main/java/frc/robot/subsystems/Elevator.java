package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.stubs.DummyMotorController;
import frc.robot.stubs.DummyRelativeEncoder;

public class Elevator implements Subsystem {

    public static final double LEVEL_1_POSITION = -4.0d;
    public static final double LEVEL_2_POSITION = 0.0d;
    public static final double LEVEL_2_ALGAE_START_POSITION = 0.0d;
    public static final double LOWER_ALGAE_POSITION = -8.0d;
    public static final double LEVEL_3_POSITION = 0.0d;
    public static final double LEVEL_3_ALGAE_START_POSITION = 0.0d;
    public static final double UPPER_ALGAE_POSITION = -12.83d;
    public static final double LEVEL_4_POSITION = 0.0d;
    public static final double CORAL_STATION_INTAKE_POSITION = 0.0d;
    public static final double NET_POSITION = -22.5d;
    public static final double BOTTOM_POSITION = 0.5d;  // This _should_ be 0.0d;

    private final MotorController leftElevatorMotor;
    private final MotorController rightElevatorMotor;

    private final SparkMaxConfig leftElevatorMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightElevatorMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder leftElevatorMotorEncoder;
    private final RelativeEncoder rightElevatorMotorEncoder;

    public Elevator() {
        if (Constants.USE_STUBS) {
            leftElevatorMotor = new DummyMotorController();
            rightElevatorMotor = new DummyMotorController();

            leftElevatorMotorEncoder = new DummyRelativeEncoder();
            rightElevatorMotorEncoder = new DummyRelativeEncoder();
        } else {
            SparkMax leftElevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_1_ID, MotorType.kBrushless);

            leftElevatorMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(60);

            leftElevatorMotorConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyClosed)
                .reverseLimitSwitchType(Type.kNormallyClosed)
                .forwardLimitSwitchEnabled(false)  // FIXME: Both of these should be true once wired up
                .reverseLimitSwitchEnabled(false);

            //leftElevatorMotorConfig.encoder
            //    .positionConversionFactor(1.0d);  // NOTE: Can change units coming out to something other than "clicks"

            leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            leftElevatorMotorEncoder = leftElevatorMotor.getEncoder();
            this.leftElevatorMotor = leftElevatorMotor;

            SparkMax rightElevatorMotor = new SparkMax(Constants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);

            rightElevatorMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(60);  // inverted does NOT work

            rightElevatorMotorConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyClosed)
                .reverseLimitSwitchType(Type.kNormallyClosed)
                .forwardLimitSwitchEnabled(false)  // FIXME: Both of these should be true once wired up
                .reverseLimitSwitchEnabled(false); // NOTE: Really want both motors hooked to same limit switches

            //rightElevatorMotorConfig.encoder
            //    .positionConversionFactor(METERS_PER_ROTATION);  // NOTE: Might wish to multiple by value to get "inches"

            rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            rightElevatorMotorEncoder = rightElevatorMotor.getEncoder();
            this.rightElevatorMotor = rightElevatorMotor;
        }
    }

    // NOTE: Negative is UP!!!!!
    public void moveUp() {
        move(-0.325d);  // FIXME: Find good speed for this
    }

    public void moveDown() {
        move(0.05d);  // FIXME: Find a good value for this
    }

    public void moveUpSlow() {
        move(-0.225d);
    }

    public void moveDownSlow() {
        move(0.0d);
    }

    // NOTE: Positive is "UP"
    public void move(double percentagePower) {
        leftElevatorMotor.set(percentagePower);  // 50
        rightElevatorMotor.set(percentagePower);  // 51
    }

    public void stop() {
        leftElevatorMotor.stopMotor();
        rightElevatorMotor.stopMotor();
    }

    // Negative UP value to hold
    public void stall() {
        move(-0.125d);
    }

    public void disableLimitSwitches() {
        enableDisableLimitSwitches(false);
    }

    public void enableLimitSwitches() {
        enableDisableLimitSwitches(true);
    }

    private void enableDisableLimitSwitches(boolean enabled) {
        if (this.leftElevatorMotor instanceof SparkMax leftElevatorMotor) {
            leftElevatorMotorConfig.limitSwitch
                .forwardLimitSwitchEnabled(enabled)
                .reverseLimitSwitchEnabled(enabled);

            leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        if (this.rightElevatorMotor instanceof SparkMax rightElevatorMotor) {
            rightElevatorMotorConfig.limitSwitch
                .forwardLimitSwitchEnabled(enabled)
                .reverseLimitSwitchEnabled(enabled);

            rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public double getLeftMotorEncoderPosition() {
        return leftElevatorMotorEncoder.getPosition();
    }

    public double getRightMotorEncoderPosition() {
        return rightElevatorMotorEncoder.getPosition();
    }

    public double getEncoderPosition() {
        return rightElevatorMotorEncoder.getPosition();  // FiXME: Average values of both?????
    }

    public void resetEncoderPosition() {
        leftElevatorMotorEncoder.setPosition(0.0d);
        rightElevatorMotorEncoder.setPosition(0.0d);
    }
}
