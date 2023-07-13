package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Autos extends SequentialCommandGroup {    
    
    
    public Autos(int cases, Swerve s_Swerve, Intake m_intake){
        System.out.printf("autos selection: %d\n", cases);
        switch (cases) {
            case 0:
            movementAuto(s_Swerve, m_intake);
            break;
            case 1:
            sampleAuto(s_Swerve);
            break;
            case 2:
            // wholeShabangAuto(s_Swerve);
            break;
        default:
        doNothingAuto(s_Swerve);
        break;
    }
}

public void doNothingAuto(Swerve s_Swerve) {
    TrajectoryConfig config =
    new TrajectoryConfig(
        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics);
        
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 0), new Translation2d(0, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(0)),
                config);
                
                var thetaController =
                new ProfiledPIDController(
                    Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
                    thetaController.enableContinuousInput(-Math.PI, Math.PI);
                    
                    SwerveControllerCommand swerveControllerCommand =
                    new SwerveControllerCommand(
                        exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
                

                addCommands(
                    new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                    swerveControllerCommand
                    );
                }
                
                
                public void sampleAuto(Swerve s_Swerve) {
                    TrajectoryConfig config =
                    new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
            
            var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);
                
                SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                    exampleTrajectory,
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
                    
                    
                    addCommands(
                        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                        swerveControllerCommand
                        );
                    }
                    
                    
        public void movementAuto(Swerve s_Swerve, Intake m_intake) {
        System.out.println("move auto");

        
        String trajectoryJSON = "PathWeaver/Paths/ScoreMove.wpilib.json";
        Trajectory trajectory = new Trajectory();
        
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("Path " + trajectoryPath);
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
                    new IntakeCommand(m_intake, 0.75, 1.0),
            swerveControllerCommand
            );
        }
        
        public void wholeShabangCenterAuto(Swerve s_Swerve, Intake m_intake) { 
            //started work for pathweaver, have to change paths still
            System.out.println("whole Shabang auto");
            //movement part 1
            String trajectoryJSON = "PathWeaver/Paths/ScoreMove.wpilib.json"; //"PathWeaver/Paths/WholeShabangCenterOne.wpilib.json";
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
                String trajectoryJSON2 = "PathWeaver/Paths/ScoreMove.wpilib.json"; //"PathWeaver/Paths/WholeShabangCenterTwo.wpilib.json";
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
            new IntakeCommand(m_intake, 0.75, 1.0),
            swerveControllerCommand, 
            swerveControllerCommand2,
            new BalanceCommand(s_Swerve, 1),
            new BalanceCommand(s_Swerve, 2)
        );
    }
}