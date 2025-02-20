package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final DutyCycleOut motorOutput = new DutyCycleOut(0);

    public ElevatorSubsystem(int motor1ID, int motor2ID) {
        motor1 = new TalonFX(motor1ID);
        motor2 = new TalonFX(motor2ID);
        
        // Create configuration object
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration motor2Config = new TalonFXConfiguration();
        motor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configurations
        motor1.getConfigurator().apply(motorConfig);
        motor2.getConfigurator().apply(motor2Config);
    }

    public void setSpeed(double speed) {
        motor1.setControl(motorOutput.withOutput(speed));
        motor2.setControl(motorOutput.withOutput(speed));
    }

    public void stop() {
        motor1.setControl(motorOutput.withOutput(0));
        motor2.setControl(motorOutput.withOutput(0));
    }
}
