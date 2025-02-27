package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

public class AutoAlignToReef extends Command {

    //These are the subsystems that the command will use.
    private final CommandSwerveDrivetrain swerve; 
    private final LimelightSubsystem limelight;

    /*
    PID CONTROLERS 
    If the robot is too far left, the PID controller will strafe right.
    If the robot is angled wrong, the PID controller will rotate correctly.
    */
    private final PIDController rotationPID = new PIDController(0, 0.0, 0.0); //PID for rotation calculates the error between the target and the current rotation
    private final PIDController strafePID = new PIDController(0.1, 0.0, 0.0); // PID for strafe calculates the error between the target and the current x position
    private final PIDController forwardPID = new PIDController(0.1, 0.0, 0.0); // PID for forward calculates the error between the target and the current y position

    /*
    AprilTags both sides (DON'T CHANGE) 
    These are the ID's of all the tags on both reefs. 
    We want to always track the ID's of the reef tags of our alliance color. 
    We do this to avoid accidentally aligning to the wrong reef.
    */
    private final int[] blueTags = {17, 18, 19, 20, 21, 22};
    private final int[] redTags = {6, 7, 8, 9, 10, 11};

    //These are the target x and y positions of where we want the robot to be at the end of the command (units in meters)
    private final double Target_X = 0.0; // WILL NEED TO CHANGE THIS LATER (LEFT/RIGHT)
    private final double Target_Y = 0.0; // WILL NEED TO CHANGE THIS LATER  (DISTANCE FROM REEF)

    //This is the request that the swerve drive will use to move the robot. Its the same command that moves our robot in teleop (X, Y and Rotation) that we want for our bot
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
    
    //constructor (This is where we actually create the command)
    public AutoAlignToReef(CommandSwerveDrivetrain swerveDrive, LimelightSubsystem limelight) {
        this.swerve = swerveDrive;
        this.limelight = limelight;
        addRequirements(swerve, limelight);
    }
    
    /*
     * We use these tolerances as we don’t really mind small errors like it being 1 degree off rotation-wise. 
     * This makes it easier to tune and honestly makes our life a lot easier. As we won’t have the robot constantly oscillating (wiggling)
     */
    @Override
    public void initialize() {
    

        rotationPID.setTolerance(1.0); //1 degree
        strafePID.setTolerance(0.05); // 5 cm
        forwardPID.setTolerance(.1); // 10 cm
    }

    // This is the main part of the command, everything in here will be constantly happening when the command is running
    @Override
    public void execute() {
        
        // Prints in the drive station if the camera can't detect a target
        if (!limelight.hasValidTarget()) {
            DriverStation.reportError("No valid target found", false);
            return;
        }

        // Prints in the drive station if the camera can see a target but it's not a reef tag
        int tagID = limelight.getTagID();
        if (!isReefTag(tagID)) {
            DriverStation.reportError("Not a reef tag", false);
            return;
        }

        double[] botPose = limelight.getBotPose();
        double x = botPose[0];
        double y = botPose[1];
        double rotation = Math.toRadians(botPose[5]);

        // We use PID to calculate movement speeds
        double rotationSpeed = rotationPID.calculate(rotation, 0.0);
        double strafeSpeed = strafePID.calculate(x, Target_X);
        double forwardSpeed = forwardPID.calculate(y, Target_Y);

        // We apply the speeds to the swerve drive
        driveRequest.withVelocityX(forwardSpeed)
                    .withVelocityY(strafeSpeed)
                    .withRotationalRate(rotationSpeed);

        // Use setControl() instead of applyRequest()
        swerve.setControl(driveRequest);
    }

    // Makes sure the robot stops when we are close enough to the target
    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint() && strafePID.atSetpoint() && forwardPID.atSetpoint();
    }

    @Override // Stops the movement of the robot
    public void end(boolean interrupted) {
        swerve.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    private boolean isReefTag(int tagID) {
        int[] validTags = blueTags; // FORCE BLUE ALLIANCE
        
        for (int tag : validTags) {
            if (tagID == tag) {
                return true;
            }
        }
        return false;
    }
}
