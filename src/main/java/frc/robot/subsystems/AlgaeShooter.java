package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeShooter extends SubsystemBase {
    private final TalonFX shooterMotor;

    public AlgaeShooter(int falconId) {
        shooterMotor = new TalonFX(falconId);
        
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterMotor.getConfigurator().apply(motorConfig);
    }

    public void setSpeed(double speed) {
        shooterMotor.set(speed);
        SmartDashboard.putNumber("Algae Shooter Speed", speed); // Debug: Check the value
    }

    public void stop() {
        shooterMotor.set(0);
    
        SmartDashboard.putNumber("Algae Shooter Speed", 0); // Debug: Ensure it stops
    }

    /** Runs shooter forward at 30% speed */
    public Command runShooterForwardCommand() {
        return this.startEnd(() -> {
            setSpeed(-0.5);
            System.out.println("Shooter Forward"); // Debug log
        }, this::stop);
    }

    /** Runs shooter in reverse at -30% speed */
    public Command runShooterReverseCommand() {
        return this.startEnd(() -> {
            setSpeed(0.5);
            System.out.println("Shooter Reverse"); // Debug log
        }, this::stop);
    }
}
