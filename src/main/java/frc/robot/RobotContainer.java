// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAlignToReef;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.MoveElevatorToHeight;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    
    // =========================
    //  DRIVE SYSTEM CONSTANTS
    // =========================
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // Max angular velocity

    // =========================
    //  SWERVE DRIVE REQUESTS
    // =========================
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Open-loop control

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // =========================
    //  SUBSYSTEMS
    // =========================
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Algae algae = new Algae(55, 56);
    private final Shooter shooter = new Shooter(15, 16);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(13, 14);
    private final LEDSubsytem led = new LEDSubsytem(0, 30);
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    // =========================
    //  CONTROLLER
    // =========================
    private final PS4Controller joystick = new PS4Controller(0);

    // =========================
    //  AUTO SELECTOR
    // =========================
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    // =========================
    //  CONSTRUCTOR
    // =========================
    public RobotContainer() {
        configureBindings();
        configureSmartDashboard();
    }

    // =========================
    //  SMARTDASHBOARD SETUP
    // =========================
    private void configureSmartDashboard() {
        SmartDashboard.putData("Run Selected Auto", new InstantCommand(() -> {
            Command autoCommand = autoChooser.getSelected();
            if (autoCommand != null) {
                autoCommand.schedule();
            } else {
                System.out.println("No auto path selected!");
            }
        }));
    }

    // =========================
    //  COMMAND BINDINGS
    // =========================
    private void configureBindings() {

        // Shooter Controls
        new JoystickButton(joystick, PS4Controller.Button.kR1.value)
                .whileTrue(shooter.runShooterForwardCommand());
        
        new JoystickButton(joystick, PS4Controller.Button.kL1.value)
                .whileTrue(shooter.runShooterReverseCommand());

        // Algae Shooter Controls (L3 & R3)
        new Trigger(() -> joystick.getL3Button())
                .whileTrue(algae.runShooterForwardCommand());

        new Trigger(() -> joystick.getR3Button())
                .whileTrue(algae.runShooterReverseCommand());

        // Auto Aligning using Limelight
        new JoystickButton(joystick, PS4Controller.Button.kCross.value)
                .whileTrue(new AutoAlignToReef(drivetrain, limelightSubsystem));

        // Elevator Controls
        elevator.setDefaultCommand(new ElevatorCommand(elevator, joystick));

        new JoystickButton(joystick, PS4Controller.Button.kShare.value)
                .onTrue(new MoveElevatorToHeight(elevator, 0));

        new JoystickButton(joystick, PS4Controller.Button.kSquare.value)
                .onTrue(new MoveElevatorToHeight(elevator, 10));

        new JoystickButton(joystick, PS4Controller.Button.kTriangle.value)
                .onTrue(new MoveElevatorToHeight(elevator, 27));

        new JoystickButton(joystick, PS4Controller.Button.kCircle.value)
                .onTrue(new MoveElevatorToHeight(elevator, 65));

        // Field-centric reset
        new JoystickButton(joystick, PS4Controller.Button.kOptions.value)
                .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // =========================
        //  DRIVETRAIN DEFAULT COMMAND
        // =========================
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)  // Forward/Backward
                     .withVelocityY(-joystick.getLeftX() * MaxSpeed)  // Strafing
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate)  // Rotation
        ));

        drivetrain.registerTelemetry(new Telemetry(MaxSpeed)::telemeterize);
    }

    // =========================
    //  GET AUTONOMOUS COMMAND
    // =========================
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    // =========================
    //  SET LED DEFAULT COLOR
    // =========================
    public Command setLEDDeafult() {
        return new InstantCommand(() -> led.setColor(57, 255, 20), led);
    }
}
