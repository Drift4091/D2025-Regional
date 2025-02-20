package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final PS4Controller controller;

    public ElevatorCommand(ElevatorSubsystem elevator, PS4Controller controller) {
        this.elevator = elevator;
        this.controller = controller;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double upSpeed = controller.getR2Axis();  
        double downSpeed = controller.getL2Axis(); 
        double speed = upSpeed - downSpeed; 

        elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
