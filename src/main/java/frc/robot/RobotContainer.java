// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
 import frc.robot.commands.AutoAlignToReef;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.MoveElevatorToHeight;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDSubsytem;

public class RobotContainer {
/* PathPlanner auto selector */



    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final PS4Controller joystick = new PS4Controller(0);
    private final ElevatorSubsystem elevator = new ElevatorSubsystem(13, 14);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    private final Shooter shooter = new Shooter(15, 16);
    
    private final LEDSubsytem led = new LEDSubsytem(0,30);
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    
    public RobotContainer() {
        configureBindings();
        

        SmartDashboard.putData("Run Selected Auto", new InstantCommand(() -> {
            Command autoCommand = autoChooser.getSelected();
            if (autoCommand != null) {
                autoCommand.schedule(); // Run auto command manually
            } else {
                System.out.println("No auto path selected!");
            }
        }));        
        
    }

    public Command setLEDDeafult(){
        return new InstantCommand(() -> led.setColor(57,255,20), led);
    }

    

    private void configureBindings() {

        new Trigger(joystick::getOptionsButtonPressed)
            .onTrue(new InstantCommand(() -> elevator.resetEncoder()));

        new JoystickButton(joystick, PS4Controller.Button.kR1.value)
        .whileTrue(shooter.runShooterForwardCommand());

        
        new JoystickButton(joystick, PS4Controller.Button.kL1.value)
            .whileTrue(shooter.runShooterReverseCommand());

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        new JoystickButton(joystick, PS4Controller.Button.kCross.value).whileTrue(new AutoAlignToReef(drivetrain, limelightSubsystem));
    
        elevator.setDefaultCommand(new ElevatorCommand(elevator,joystick ));
        
        // new JoystickButton(joystick, PS4Controller.Button.kCross.value).whileTrue(drivetrain.applyRequest(() -> brake));
        
        // new JoystickButton(joystick, PS4Controller.Button.kCircle.value)
        // .whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // new JoystickButton(joystick, PS4Controller.Button.kShare.value).and(new JoystickButton(joystick, PS4Controller.Button.kTriangle.value)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

       

        new JoystickButton(joystick, PS4Controller.Button.kOptions.value).and(new JoystickButton(joystick, PS4Controller.Button.kTriangle.value)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        new JoystickButton(joystick, PS4Controller.Button.kOptions.value).and(new JoystickButton(joystick, PS4Controller.Button.kSquare.value)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    //     new JoystickButton(joystick, PS4Controller.Button.kSquare.value)
    // .onTrue(new MoveElevatorToHeight(elevator,8));

    new JoystickButton(joystick, PS4Controller.Button.kTriangle.value)
    .onTrue(new MoveElevatorToHeight(elevator, 27));
    
    new JoystickButton(joystick, PS4Controller.Button.kCircle.value)
    .onTrue(new MoveElevatorToHeight(elevator, 65));
    
    
        // Reset field-centric heading on L1 press
    new JoystickButton(joystick, PS4Controller.Button.kShare.value)
        .onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));  drivetrain.registerTelemetry(logger::telemeterize);
     }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
