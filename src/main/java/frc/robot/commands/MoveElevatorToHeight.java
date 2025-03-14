package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorToHeight extends Command {
  
    private final ElevatorSubsystem elevator;
    private final double targetHeight;
    private final double kP = 0.2; // Tune this value as needed
    private final double tolerance = 0.5; // Stop when within 1 unit

    public MoveElevatorToHeight(ElevatorSubsystem evilElevator, double evilTarget) {
        elevator = evilElevator;
        targetHeight = Math.max(1, Math.min(evilTarget, 72)); // Clamp between limits
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
        // speed = Math.max(-0.5, Math.min(speed, 0.5));
        // if (elevator.getBottomLimitSwitch() && speed < 0){
        //     speed = 0;
        // } else if (elevator.getTopLimitSwitch() && speed > 0){
        //     speed = 0;
        // } else {
        // elevator.setSpeed(speed);

        speed = Math.max(-0.5, Math.min(speed, 0.5));
        if (speed < 0){
            speed = 0;
        } else if (speed > 0){
            speed = 0;
        } else {
        elevator.setSpeed(speed);
    }
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
