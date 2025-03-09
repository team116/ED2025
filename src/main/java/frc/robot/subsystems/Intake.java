package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.stubs.DummyMotorController;

public class Intake implements Subsystem {
    private final MotorController leftIntakeMotor;
    private final MotorController rightIntakeMotor;

    private final SparkMaxConfig leftIntakeMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightIntakeMotorConfig = new SparkMaxConfig();

    private final DigitalInput coralContactLimitSwitch;
    private final DigitalInput algaeContactLimitSwitch;

    public Intake() {
        if (Constants.USE_STUBS) {
            leftIntakeMotor = new DummyMotorController();
            rightIntakeMotor = new DummyMotorController();
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
        }

        this.algaeContactLimitSwitch = new DigitalInput(Constants.ALGAE_CONTACT_SWITCH_CHANNEL);
        this.coralContactLimitSwitch = new DigitalInput(Constants.CORAL_CONTACT_SWITCH_CHANNEL);
    }

    public void consume() {
        leftIntakeMotor.set(-0.3);
        rightIntakeMotor.set(-0.3);
    }

    public void expel() {
        leftIntakeMotor.set(0.8);
        rightIntakeMotor.set(0.8);
    }

    public void stop() {
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
