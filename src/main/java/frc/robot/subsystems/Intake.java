package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class Intake implements Subsystem {
    private final SparkMax leftIntakeMotor;
    private final SparkMax rightIntakeMotor;

    private final SparkMaxConfig leftIntakeMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightIntakeMotorConfig = new SparkMaxConfig();

    private final DigitalInput coralContactLimitSwitch;
    private final DigitalInput algaeContactLimitSwitch;

    public Intake() {
        leftIntakeMotor = new SparkMax(Constants.INTAKE_MOTOR_1_ID, MotorType.kBrushless);

        leftIntakeMotorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(20);

        leftIntakeMotorConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchType(Type.kNormallyClosed)
            .forwardLimitSwitchEnabled(false)
            .reverseLimitSwitchEnabled(false);

        leftIntakeMotor.configure(leftIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightIntakeMotor = new SparkMax(Constants.INTAKE_MOTOR_2_ID, MotorType.kBrushless);

        rightIntakeMotorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(20);

        rightIntakeMotorConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyClosed)
        .reverseLimitSwitchType(Type.kNormallyClosed)
        .forwardLimitSwitchEnabled(false)  // FIXME: Both of these should be true once wired up
        .reverseLimitSwitchEnabled(false);

        rightIntakeMotor.configure(rightIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.algaeContactLimitSwitch = new DigitalInput(Constants.ALGAE_CONTACT_SWITCH_CHANNEL);
        this.coralContactLimitSwitch = new DigitalInput(Constants.CORAL_CONTACT_SWITCH_CHANNEL);
    }

    public void consume() {
        leftIntakeMotor.set(-0.50);
        rightIntakeMotor.set(0.50);
    }

    public void expel() {
        leftIntakeMotor.set(0.20);
        rightIntakeMotor.set(-0.20);
    }

    public void launch() {
        leftIntakeMotor.set(0.55);
        rightIntakeMotor.set(-0.55);
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

    public void printMotorVoltages() {
        SmartDashboard.putNumber("ID55 V", leftIntakeMotor.getBusVoltage());
        SmartDashboard.putNumber("ID56 V", rightIntakeMotor.getBusVoltage());
    }
}
