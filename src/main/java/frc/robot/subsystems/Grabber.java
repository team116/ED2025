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

/**
 * ## Grabber / Intake / THE Manipulator (combined subsystems?)
### Hand / Claw
All REVLib
- 2 NEO Motors
- Left Motor CAN ID: 55
- Right Motor CAN ID: 56
- coral contact limit switch
- algae contact limit switch

### Wrist
All REVLib (?? or is there a CANCoder or other type of abs encoder ??)
- 1 NEO Motor
- Motor CAN ID: 57
- Probably CANCoder
- CAN Coder CAN ID: 58
- 2 max ranges limit switches (safety)
 */
public class Grabber implements Subsystem {
    private final MotorController leftIntakeMotor;
    private final MotorController rightIntakeMotor;

    private final MotorController wristMotor;
    private final RelativeEncoder wristMotorEncoder;
    private final CANcoder wristCANCoder;
    private final CANcoderConfiguration wristCANCoderConfig = new CANcoderConfiguration();

    private final SparkMaxConfig leftIntakeMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightIntakeMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();

    private final DigitalInput coralContactLimitSwitch;
    private final DigitalInput algaeContactLimitSwitch;

    public Grabber() {
        if (Constants.USE_STUBS) {
            leftIntakeMotor = new DummyMotorController();
            rightIntakeMotor = new DummyMotorController();
            wristMotor = new DummyMotorController();
            wristMotorEncoder = new DummyRelativeEncoder();
            wristCANCoder = null;  // FIXME: Cannot dummy this out because no interface BOO!!!!!
        } else {
            SparkMax leftIntakeMotor = new SparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);

            leftIntakeMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false);

            leftIntakeMotor.configure(leftIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            this.leftIntakeMotor = leftIntakeMotor;

            SparkMax rightIntakeMotor = new SparkMax(Constants.INTAKE_MOTOR_2_ID, MotorType.kBrushless);

            rightIntakeMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false);

            rightIntakeMotor.configure(rightIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            this.rightIntakeMotor = rightIntakeMotor;

            SparkMax wristMotor = new SparkMax(Constants.WRIST_MOTOR_ID, MotorType.kBrushless);

            wristMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false);

            wristMotorConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyClosed)
                .reverseLimitSwitchType(Type.kNormallyClosed)
                .forwardLimitSwitchEnabled(false)  // FIXME: Both of these should be true once wired up
                .reverseLimitSwitchEnabled(false);

            // FIXME: Might want this to be a factor to change outputs to angles
            //wristMotorConfig.encoder
            //    .positionConversionFactor(1.0d);

            wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            wristMotorEncoder = wristMotor.getEncoder();
            this.wristMotor = wristMotor;

            wristCANCoder = new CANcoder(Constants.WRIST_CANCODER_ID);
            wristCANCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5d;
            wristCANCoder.getConfigurator().apply(wristCANCoderConfig);
        }

        this.algaeContactLimitSwitch = new DigitalInput(Constants.ALGAE_CONTACT_SWITCH_CHANNEL);
        this.coralContactLimitSwitch = new DigitalInput(Constants.CORAL_CONTACT_SWITCH_CHANNEL);
    }

    public double getWristRelativePosition() {
        return wristMotorEncoder.getPosition();
    }

    public double getWristAbsolutePosition () {
        if (wristCANCoder == null) {
            return 0.0d;
        }

        return wristCANCoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getWristAbsoluteAngle() {
        return getWristAbsolutePosition() * 180.0d;
    }

    public void wristUp() {
        wristMotor.set(0.1);
    }

    public void wristDown() {
        wristMotor.set(-0.1);
    }

    public void stopWristMotors() {
        wristMotor.stopMotor();
    }

    public void runIntakeMotorsToConsume() {
        leftIntakeMotor.set(-0.1);
        rightIntakeMotor.set(-0.1);
    }

    public void runIntakeMotorsToExpel() {
        leftIntakeMotor.set(0.5);
        rightIntakeMotor.set(0.5);
    }

    public void stopIntakeMotors() {
        leftIntakeMotor.stopMotor();
        rightIntakeMotor.stopMotor();
    }

    public boolean algaeLimitSwitchIsPressed() {
        return algaeContactLimitSwitch.get();
    }

    public boolean coralLimitSwitchIsPressed() {
        return coralContactLimitSwitch.get();
    }
}
