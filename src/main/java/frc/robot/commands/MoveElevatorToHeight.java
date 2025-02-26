package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorToHeight extends Command {
  
    private final ElevatorSubsystem elevator;
    private final double targetHeight;
    private final double kP = 0.01; // Tune this value as needed
    private final double tolerance = 3.0; // Stop when within 1 unit

    public MoveElevatorToHeight(ElevatorSubsystem evilElevator, double evilTarget) {
        elevator = evilElevator;
        targetHeight = Math.max(0.5, Math.min(evilTarget, 72)); // Clamp between limits
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        System.out.println("MoveElevatorToHeight started: Target = " + targetHeight);
    }

    @Override
    public void execute() {
        double currentHeight = elevator.getEncoderPosition();
        double error = targetHeight - currentHeight;
        double speed = kP * error; 

        // Clamp speed to safe limits
        speed = Math.max(-0.25, Math.min(speed, 0.25));

        elevator.setSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(targetHeight - elevator.getEncoderPosition()) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("MoveElevatorToHeight ended.");
        elevator.stop();
    }
}
