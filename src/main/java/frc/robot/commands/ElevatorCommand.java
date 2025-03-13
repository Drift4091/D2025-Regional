package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final PS4Controller controller;
    private final double holdingForce = 0.012;

    public ElevatorCommand(ElevatorSubsystem elevator, PS4Controller controller) {
        this.elevator = elevator;
        this.controller = controller;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double downSpeed;
        double upSpeed;
        SmartDashboard.putBoolean("Bottom Limit Switch", elevator.getBottomLimitSwitch());

        if ( elevator.getEncoderPosition() < 1 || elevator.getBottomLimitSwitch()) {
            downSpeed = 0;
        } else {
            downSpeed = (controller.getL2Axis() + 1) / -4;
        }

        if ( elevator.getEncoderPosition() > 80) {
            upSpeed = 0;
        } else {
            upSpeed = (controller.getR2Axis() + 1) / 4;
            if (Math.abs(upSpeed) < holdingForce) {
                upSpeed = holdingForce;
            }
        }


        double speed = upSpeed + downSpeed;
        elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        
    }
}
