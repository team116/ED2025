package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.stubs.DummyMotorController;
import frc.robot.stubs.DummyRelativeEncoder;

public class Climber implements Subsystem {
    private final MotorController climberMotor;
    private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
    private final RelativeEncoder climberMotorEncoder;

    public Climber() {
        if (Constants.USE_STUBS) {
            climberMotor = new DummyMotorController();
            climberMotorEncoder = new DummyRelativeEncoder();
        } else {
            SparkMax climberMotor = new SparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);

            climberMotorConfig
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(40);

            climberMotorConfig.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyClosed)
                .reverseLimitSwitchType(Type.kNormallyClosed)
                .forwardLimitSwitchEnabled(false)  // FIXME: Set to true for correct one...
                .reverseLimitSwitchEnabled(false);

            climberMotor.configure(climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            climberMotorEncoder = climberMotor.getEncoder();
            this.climberMotor = climberMotor;
        }
    }

    public double getEncoderPosition() {
        return climberMotorEncoder.getPosition();
    }

    public double getMotorPower() {
        return climberMotor.get();
    }

    public void setMotorPower(double percentagePower) {
        climberMotor.set(percentagePower);
    }

    public void pullUp() {
        setMotorPower(1.0);
    }

    public void pullUpSlow() {
        setMotorPower(0.5);
    }


    public void dropDown() {
        setMotorPower(-1.0);
    }

    public void dropDownSlow() {
        setMotorPower(-0.5);
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public void disableLimitSwitches() {
        if (this.climberMotor instanceof SparkMax climberMotor) {
            climberMotorConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);

            climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }

    public void enableLimitSwitches() {
        if (this.climberMotor instanceof SparkMax climberMotor) {
            climberMotorConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)  // FIXME: only one of these should be set to true
                .reverseLimitSwitchEnabled(false);

            climberMotor.configure(climberMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }
    }
}
