package frc.robot.controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.DiffDriveForDurationCommand;
import frc.robot.utils.SwerveDriveForDurationCommand;
import frc.robot.commands.SwerveSampleMoveCommand;

public class AutonController {
    public static CommandBase getAutonCommand() {
        return DriveConstants.driveType.equals("DIFFER")? getDiffAutonomousCommand() : getSimpleSwerveCommand();
    }

    private static CommandBase getSimpleSwerveCommand() {
        System.out.println("getSwerveAutonomousCommand called");
        //return null;
        SwerveDriveSubsystem s_drive = SwerveDriveSubsystem.getInstance();
        s_drive.setMaxSpeeds(0.5, 0.5);

        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

         //move forward
         commandGroup.addCommands(new SwerveSampleMoveCommand(s_drive, 1, 0, 0, true, new Pose2d(), 0.05));

        /*
        //Z shape
        //move left
        commandGroup.addCommands(new SwerveSampleMoveCommand(s_drive, 0, -1, 0, true, new Pose2d(), 0.05));
        //swerve 45 degrees right to back
        commandGroup.addCommands(new SwerveSampleMoveCommand(s_drive, -1, 1, 0, true, new Pose2d(), 0.05));
        //move left
        commandGroup.addCommands(new SwerveSampleMoveCommand(s_drive, 0, -1, 0, true, new Pose2d(), 0.05));
        

        //reverse Z shape
        //move left
        commandGroup.addCommands(new SwerveSampleMoveCommand(s_drive, 0, 1, 0, true, new Pose2d(), 0.05));
        //swerve 45 degrees right to back
        commandGroup.addCommands(new SwerveSampleMoveCommand(s_drive, 1, -1, 0, true, new Pose2d(), 0.05));
        //move left
        commandGroup.addCommands(new SwerveSampleMoveCommand(s_drive, 0, 1, 0, true, new Pose2d(), 0.05));

*/
        return commandGroup;
    }

    private static CommandBase getSwerveWithDurationsAutonCommand() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0.05,0,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,-0.05,0,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0,0.05,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0,-0.05,0.0));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0,0,0.2));
        commandGroup.addCommands(new SwerveDriveForDurationCommand(2,0.1,0.1,0.0));

       // commandGroup.addCommands(SwerveDriveSubsystem.getInstance().driveTimeCommand(5, 0.05, 0.05, 0.0, true, true));
        return commandGroup;
    }

    private static CommandBase getDiffAutonomousCommand() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DiffDriveForDurationCommand(2,0.2,0.0));
        commandGroup.addCommands(DifferentialDriveSubsystem.getInstance().driveTimeCommand(3, 0.3, 0.4));
        return commandGroup;
    }
}