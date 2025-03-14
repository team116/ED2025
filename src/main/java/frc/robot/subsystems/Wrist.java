package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.stubs.DummyMotorController;
import frc.robot.stubs.DummyRelativeEncoder;

public class Wrist implements Subsystem {

    private static final double WRIST_GEARBOX_GEAR_RATIO = 36.0d/1.0d;  // 36:1
    private static final double WRIST_DEGREES_PER_REVOLUTION = 360.0d / WRIST_GEARBOX_GEAR_RATIO;

    public static final double WRIST_CORAL_STATION_INTAKE_ANGLE = 120.0d;
    public static final double WRIST_LEVEL_4_NEUTRAL_ANGLE = 170.0d;
    public static final double WRIST_LEVEL_2_AND_3_ANGLE = 130.0d;
    public static final double WRIST_DOWN_FULL_ANGLE = 250.0d;
    public static final double WRIST_TROUGH_LEVEL_FOR_AUTO = 142.0d;

    private final MotorController wristMotor;
    private final RelativeEncoder wristMotorEncoder;
    private final CANcoder wristCANCoder;
    private final CANcoderConfiguration wristCANCoderConfig = new CANcoderConfiguration();

    private final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

    public Wrist() {
        if (Constants.USE_STUBS) {
            wristMotor = new DummyMotorController();
            wristMotorEncoder = new DummyRelativeEncoder();
            wristCANCoder = null;  // FIXME: Cannot dummy this out because no interface BOO!!!!!
        } else {
            SparkMax wristMotor = new SparkMax(Constants.WRIST_MOTOR_ID, MotorType.kBrushless);

            wristMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(true)
                .smartCurrentLimit(40);

            wristMotorConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyClosed)
                .reverseLimitSwitchType(Type.kNormallyClosed)
                .forwardLimitSwitchEnabled(false)  // FIXME: Both of these should be true once wired up
                .reverseLimitSwitchEnabled(false);

            // FIXME: Might want this to be a factor to change outputs to angles
            //wristMotorConfig.encoder
                //.positionConversionFactor(WRIST_DEGREES_PER_REVOLUTION);

            wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            wristMotorEncoder = wristMotor.getEncoder();
            this.wristMotor = wristMotor;

            wristCANCoder = new CANcoder(Constants.WRIST_CANCODER_ID);
            wristCANCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5d;
            wristCANCoder.getConfigurator().apply(wristCANCoderConfig);
        }
    }

    public double getRelativePosition() {
        return wristMotorEncoder.getPosition();
    }

    public double getRelativeAngle() {
        return getRelativePosition() * WRIST_DEGREES_PER_REVOLUTION;
    }

    public double getAbsolutePosition () {
        if (wristCANCoder == null) {
            return 0.0d;
        }

        return wristCANCoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getAbsoluteAngle() {
        return getAbsolutePosition() * 180.0d;
    }

    public void up() {
        wristMotor.set(-0.16);
    }

    public void down() {
        wristMotor.set(0.08);
    }

    public void upSlow() {
        wristMotor.set(-0.06);
    }

    public void downSlow() {
        wristMotor.set(0.04);
    }

    public void stall() {
        wristMotor.set(-0.02);
    }

    public void stop() {
        wristMotor.stopMotor();
    }

    // NOTE: Assuming "top" vertical location, set to 90.0 degrees so "bottom" should be 270.0 degrees
    public void resetRelativeEncoder() {
        wristMotorEncoder.setPosition(90.0d/WRIST_DEGREES_PER_REVOLUTION);
    }
}
