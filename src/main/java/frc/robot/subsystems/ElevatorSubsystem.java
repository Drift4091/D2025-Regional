package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX motor1;
    private final TalonFX motor2;
    
    private final DutyCycleOut motorOutput = new DutyCycleOut(0);
    private final DigitalInput bottomLimitSwitch = new DigitalInput(0); // DIO port 0

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

    public void resetEncoder() {
        System.out.println("Resetting encoder...");
        motor1.setPosition(0); // Reset encoder
    }
    public void setSpeed(double speed) {
        motor1.set(speed);
        motor2.set(speed);
    }

    public double getEncoderPosition() {
        StatusSignal<Angle> positionSignal = motor1.getPosition();
        double encoderValue = positionSignal.getValueAsDouble();
        SmartDashboard.putNumber("Elevator Encoder", encoderValue);
        return encoderValue;
    }

    public boolean getBottomLimitSwitch() {
        boolean isPressed = bottomLimitSwitch.get(); // Read the switch state
        SmartDashboard.putBoolean("Bottom Limit Switch", isPressed);
        return isPressed; // Output to SmartDashboard/Shuffleboard
    }

    public void stop() { 
        motor1.set(0);
        motor2.set(0);
    }

    @Override
    public void periodic() {
        // Update the limit switch status every cycle
        getBottomLimitSwitch();
        
    }
}
