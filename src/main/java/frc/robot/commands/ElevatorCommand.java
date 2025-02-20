package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final PS4Controller controller;
    DigitalInput bottomlimitSwitch = new DigitalInput(0);
    DigitalInput toplimitSwitch = new DigitalInput(1);

    public ElevatorCommand(ElevatorSubsystem elevator, PS4Controller controller) {
        this.elevator = elevator;
        this.controller = controller;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double downSpeed;
        double upSpeed;
        
        if (bottomlimitSwitch.get()){
            downSpeed = 0;
        } else {
            downSpeed = controller.getL2Axis()/3; 
        }

        if (toplimitSwitch.get()){
            upSpeed = 0;
        } else {
            upSpeed = controller.getR2Axis()/3;
        }

        double speed = upSpeed - downSpeed; 

        elevator.setSpeed(speed);
    } 

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
