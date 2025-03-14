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

public class Algae extends SubsystemBase {
    private final SparkMax shooterMotorOne;
    private final TalonFX shooterMotorTwo;

    public Algae(int shooterMotorOneID, int shooterMotorTwoID) {
        shooterMotorOne = new SparkMax(shooterMotorOneID, MotorType.kBrushless);
        shooterMotorTwo = new TalonFX(shooterMotorTwoID);
        
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        shooterMotorTwo.getConfigurator().apply(motorConfig);
    }

    public void setSpeed(double speed) {
        shooterMotorOne.set(speed);
        shooterMotorTwo.set(speed);
        SmartDashboard.putNumber("Shooter Speed", speed); // Debug: Check the value
    }

    public void stop() {
        shooterMotorOne.set(0);
        shooterMotorTwo.set(0);
        SmartDashboard.putNumber("Shooter Speed", 0); // Debug: Ensure it stops
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
