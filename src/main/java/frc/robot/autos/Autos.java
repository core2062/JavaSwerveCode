package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Autos extends SequentialCommandGroup {    
    
   
public Autos(int cases, Swerve s_Swerve){
    switch (cases) {
        case 0:
            movementAuto(s_Swerve);
            break;
        case 1:
            wholeShabangAuto(s_Swerve);
            break;
        default:
            break;
    }
}


public void movementAuto(Swerve s_Swerve) {
        String trajectoryJSON = "paths/move.wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
     } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
     };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
                
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                Trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(Trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }

    public void wholeShabangAuto(Swerve s_Swerve) {
        //movement part 1
        String trajectoryJSON = "paths/move.wpilib.json";
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        };
        // An example trajectory to follow.  All units in meters.
        Trajectory Trajectory = trajectory;
                
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                Trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


            //movement part 2
            String trajectoryJSON2 = "paths/move.wpilib.json";
            Trajectory trajectory2 = new Trajectory();
        
            try {
                Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
                trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON2, ex.getStackTrace());
            };
            
            Trajectory Trajectory2 = trajectory2;
                        
            var thetaController2 =
                new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
            SwerveControllerCommand swerveControllerCommand2 =
                new SwerveControllerCommand(
                    Trajectory2,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController2,
                    s_Swerve::setModuleStates,
                    s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(Trajectory.getInitialPose())),
            swerveControllerCommand,
            swerveControllerCommand2
        );
    }
}