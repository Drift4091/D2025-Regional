package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.led.LEDSubsystem; // Update to the correct package if needed
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final LEDSubsystem leds;
    private final PS4Controller controller;
    private final double holdingForce = 0.01;

    public ElevatorCommand(ElevatorSubsystem elevator, LEDSubsystem leds, PS4Controller controller) {
        this.elevator = elevator;
        this.leds = leds;
        this.controller = controller;
        addRequirements(elevator, leds);
    }

    @Override
    public void initialize() {
        leds.setColor(255, 0, 0); 
    }

    @Override
    public void execute() {
        double downSpeed;
        double upSpeed;
        SmartDashboard.putBoolean("Bottom Limit Switch", elevator.getBottomLimitSwitch());
        SmartDashboard.putBoolean("Top Limit Switch", elevator.getTopLimitSwitch());

        if (elevator.getBottomLimitSwitch() || elevator.getEncoderPosition() < 1) {
            downSpeed = 0;
        } else {
            downSpeed = (controller.getL2Axis() + 1) / -4;
        }

        if (elevator.getTopLimitSwitch() || elevator.getEncoderPosition() > 71) {
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
        leds.setDefaultColor(); // Reset LEDs back to neon green when elevator stops
    }
}
