package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    // private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Joystick buttonBoard = new Joystick(1);
    Timer timer = new Timer();
    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    // private final JoystickButton rightTrigger = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
    // private final JoystickButton leftTrigger = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);

    // Trigger leftTrigger1 = new Trigger(() -> rightTrigger.getAsBoolean() > 0.5);
    // Trigger rightTrigger1 = new Trigger(() -> driver.getRightTriggerAxis() > 0.5);
    /* Driver Buttons */
    // private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);
    // private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    // private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    // private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    // private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    // private final JoystickButton rightMenu = new JoystickButton(driver, XboxController.Button.kStart.value);
    // private final JoystickButton leftMenu = new JoystickButton(driver, XboxController.Button.kBack.value);

    // private final POVButton dUp1 = new POVButton(driver, 0);
    // private final POVButton dRight1 = new POVButton(driver, 90);
    // private final POVButton dDown1 = new POVButton(driver, 180);
    // private final POVButton dLeft1 = new POVButton(driver, 270);

    final JoystickButton b1 = new JoystickButton(buttonBoard, 1);
    final JoystickButton b2 = new JoystickButton(buttonBoard, 2);
    final JoystickButton b3 = new JoystickButton(buttonBoard, 3);
    final JoystickButton b4 = new JoystickButton(buttonBoard, 4);
    final JoystickButton b5 = new JoystickButton(buttonBoard, 5);
    final JoystickButton b6 = new JoystickButton(buttonBoard, 6);
    final JoystickButton b7 = new JoystickButton(buttonBoard, 7);
    final JoystickButton b8 = new JoystickButton(buttonBoard, 8);
    final JoystickButton b9 = new JoystickButton(buttonBoard, 9);
    final JoystickButton b10 = new JoystickButton(buttonBoard, 10);
    final JoystickButton b11 = new JoystickButton(buttonBoard, 11);
    final JoystickButton b12 = new JoystickButton(buttonBoard, 12);

    Trigger bbStickF = new Trigger(() -> buttonBoard.getRawAxis(1) > 0.7);
    Trigger bbStickB = new Trigger(() -> buttonBoard.getRawAxis(1) < -0.7);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter shooterSubsystem = new Shooter();
    private final Intake intakeSubsystem = new Intake();
    private final LED LEDSubsystem = new LED();

    private final SendableChooser<Command> chooser = new SendableChooser<>();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.resetModulesToAbsolute();
        s_Swerve.resetModulesToAbsolute();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis)
                //() -> {return false;}
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        chooser.setDefaultOption("One Note Auto", oneNoteAuto());
        // chooser.addOption("S trajectory", followTrajectory(sTrajectory));
        // chooser.addOption("left trajectory", followTrajectory(leftTrajectory));
        // chooser.addOption("right trajectory", followTrajectory(rightTrajectory));
        // chooser.addOption("left timing", timingDriveLeftSide());
        // chooser.addOption("right timing", timingDriveRightSide());
        chooser.addOption("pathFollowingLeft", 
            new PathPlannerAuto("twoNoteAuto")
            .finallyDo(() -> {s_Swerve.zeroHeading();})
            );
        chooser.addOption("One Note and Leave", leftAndLeave());
        chooser.addOption("straightTest", 
            new PathPlannerAuto("1mStraightPathAuto")
            .finallyDo(() -> {s_Swerve.zeroHeading();})
            );
        chooser.addOption("ReturnToSpeaker", 
            new PathPlannerAuto("returnAuto")
            .finallyDo(() -> {s_Swerve.zeroHeading();})
            );
        SmartDashboard.putData("Auto choices", chooser);
        chooser.addOption("Event Marker Test",
            new PathPlannerAuto("eventMarkerTest"));
        chooser.addOption("Event Marker Return",
            new PathPlannerAuto("eventMarkerReturn"));

        NamedCommands.registerCommand("transform", transform());
        NamedCommands.registerCommand("intake", smartIntake());
        NamedCommands.registerCommand("shoot", shooterSubsystem.setShooterSpeed(1));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        driver.x().onTrue(transform());

        // Stow Shooter and Intake
        driver.a().onTrue(stowParallel());
        
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // driver.back().whileTrue(intakeSubsystem.lowerIntake());
        // driver.start().whileTrue(intakeSubsystem.raiseIntake());
        driver.b().onTrue(shooterSubsystem.amp());

        //WIP
        // driver.y().onTrue(shooterSubsystem.setIndexerSpeed(0.8));
        // yButton.whileTrue(
        //     shooterSubsystem.setIndexerSpeed(0.8)
        //     //.until(() ->{return shooterSubsystem.TOF.getRange() > 300;})
        //     //.andThen(shooterSubsystem.setIndexerSpeed(0))
        //     );
        
        driver.rightTrigger(0.5).whileTrue(
            shooterSubsystem.setShooterSpeed(1)
            .until(() -> {return shooterSubsystem.leftShooterMotor.getVelocity().getValue() > 100;})
            .andThen(new ParallelCommandGroup(shooterSubsystem.setIndexerSpeedNoFinallyDo(.8), LEDSubsystem.setRainbowCommand()))
            .finallyDo(() -> {
                shooterSubsystem.load(0);
                shooterSubsystem.shoot(0);
            })
            );
            
        driver.leftTrigger(.2).whileTrue(smartIntake());
        // controller.rightTrigger(.5).whileTrue(shooterSubsystem.setIndexerSpeed(.8));
        // controller.leftTrigger(.5).whileTrue( smartIntake());\

        driver.leftBumper().whileTrue(new InstantCommand(() -> {shooterSubsystem.activateRatchet();}));
        driver.leftBumper().onTrue(LEDSubsystem.setOrangeCommand());
        driver.rightBumper().whileTrue(smartClimb());
        driver.rightBumper().onTrue(LEDSubsystem.setRainbowCommand());

    }

    public Command transform() {
        return shooterSubsystem.deploy()
            .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() > 0.13;}) //0.25
            .andThen(intakeSubsystem.deploy())
            .until(() -> {return intakeSubsystem.angleEncoder.getAbsolutePosition().getValue() > 0.33;})
            .andThen(shooterSubsystem.stow())
            // .andThen(new InstantCommand(() -> {}))
            .withName("OUT (TRANSFORM)");
            // .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.05;});
    }

    public Command stow() {
        return shooterSubsystem.deploy()
            .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() > 0.13;})
            // .andThen(new ParallelCommandGroup())
            .andThen(intakeSubsystem.stow())
            .until(() -> {return intakeSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.05;})
            .andThen(shooterSubsystem.stow());
            // .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.05;})
    }

    public Command stowParallel() {
        return new ParallelCommandGroup(shooterSubsystem.deploy(), intakeSubsystem.stow())
            .until(() -> {return intakeSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.05;}) // 0.05 // WIP
            .andThen(shooterSubsystem.stow());
            // .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.05;})
    }

    public Command indexWithTOF() {
        return shooterSubsystem.setIndexerSpeed(-0.05)
            .until(() -> {return shooterSubsystem.TOF.getRange() > 165;})
            .andThen(new ParallelCommandGroup(shooterSubsystem.setIndexerSpeedRunOnce(0), LEDSubsystem.setGreenCommand()))
            .withName("INDEX NOTE WITH TOF");
    }

    public Command smartClimb() {
           return shooterSubsystem.climb()
            // .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.03;})
            .finallyDo(() -> {shooterSubsystem.deactivateRatchet();})
            .withName("Climbing");
    }

    public Command smartIntake() {
        return  
            new ParallelCommandGroup(
                shooterSubsystem.setIndexerSpeed(1),
                intakeSubsystem.setIntakeSpeed(0.8),
                new InstantCommand(() -> {shooterSubsystem.setAnglePosition(0.0225);}) // 0.0125
                )
            .until(() -> {return shooterSubsystem.TOF.getRange() < 165;})
            .andThen(new InstantCommand(() -> {shooterSubsystem.load(0);}))
            .andThen(new InstantCommand(() -> {intakeSubsystem.load(0);}))
            .andThen(indexWithTOF())
            .finallyDo(() -> {
                intakeSubsystem.load(0);
                shooterSubsystem.setAnglePosition(0);
                if (shooterSubsystem.TOF.getRange() > 165){
                    LEDSubsystem.red();
                } else if (shooterSubsystem.TOF.getRange() > 110){
                    // shooterSubsystem.lowerNote();
                    LEDSubsystem.yellow();
                } else {
                    LEDSubsystem.GreenFlow();
                }
            });
            // .andThen(shooterSubsystem.setIndexerSpeed(-0.05))
            // .until(() -> {return shooterSubsystem.TOF.getRange() > 165;})
            // .andThen(shooterSubsystem.setIndexerSpeed(0))
            // .andThen(shooterSubsystem.setIndexerSpeed(0))
            // .andThen(intakeSubsystem.setIntakeSpeed(0))
    }

        // Create config for trajectory
        // TrajectoryConfig config =
        //     new TrajectoryConfig(
        //             1,
        //             1)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(Constants.Swerve.swerveKinematics);
        //         // Apply the voltage constraint
        //         // .addConstraint(autoVoltageConstraint);

        // // An example trajectory to follow. All units in meters.
        // Trajectory sTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(3, 0, new Rotation2d(0)),
        //         // Pass config
        //         config);

        // Trajectory leftTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(1.78, 6.74, new Rotation2d(55)),
        //         // Pass through these interior waypoints
        //         List.of(new Translation2d(1.9, 6.93)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(2.9, 6.98, new Rotation2d(0)),
        //         // Pass config
        //         config);

        // Trajectory rightTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0.66, 4.38, new Rotation2d(-52.55)),
        //         // Pass through these interior waypoints
        //         List.of(new Translation2d(1.37, 3.73)),
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(2.43, 3.86, new Rotation2d(26.05)),
        //         // Pass config
        //         config);

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return oneNoteAuto();
        // return new PathPlannerAuto("twoNoteAuto");
        // return followTrajectory(sTrajectory);
        return chooser.getSelected();
    }

    // public Command followTrajectory(Trajectory trajectory) {
    //     ProfiledPIDController thetaController = new ProfiledPIDController(15, 0, 0, AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     HolonomicDriveController controller = new HolonomicDriveController(
    //             new PIDController(2.75, 0, 0.5), 
    //             new PIDController(2.75, 0, 0.5), 
    //             thetaController
    //         );

    //     SwerveControllerCommand swerveControllerCommand =
    //         new SwerveControllerCommand(trajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, controller, s_Swerve::setModuleStates, s_Swerve);
                
    //     return Commands.runOnce(
    //         () -> s_Swerve.setPose(trajectory.getInitialPose()))
    //         .andThen(swerveControllerCommand)
    //         .andThen(Commands.runOnce(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, false)));
    // }
        

    public Command oneNoteAuto() {
        return new InstantCommand(() -> {timer.restart();})
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .until(() -> {return timer.get() > 1;})
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return timer.get() > 2;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0))
            .andThen(transform())
            .until(() -> {return timer.get() > 4;});
            // .andThen(
            //     new ParallelCommandGroup(smartIntake(),
            //     new PathPlannerAuto("twoNoteAuto"))
            //     );

    }

    public Command leftAndLeave() {
        return new InstantCommand(() -> {timer.restart();})
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .until(() -> {return timer.get() > 1;})
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return timer.get() > 2;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0))
            .andThen(transform())
            .until(() -> {return timer.get() > 4;})
            .andThen(
                new ParallelCommandGroup(smartIntake(),
                new PathPlannerAuto("twoNoteAuto"))
                )
            .until(() -> {return timer.get() > 7;})
            .andThen(new PathPlannerAuto("returnAuto"))
            .until(() -> {return timer.get() > 10;})
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .until(() -> {return timer.get() > 11;})
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return timer.get() > 12;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));

    }

    // public Command timingDriveLeftSide() {
    //     return new InstantCommand(() -> {timer.restart();})
    //         .andThen(new RunCommand(()->{s_Swerve.driveLeft();}, s_Swerve))
    //         .until(() -> {return timer.get() > 1;})
    //         .andThen(new RunCommand(()->{s_Swerve.driveForward();}, s_Swerve))
    //         .until(() -> {return timer.get() > 3;});
    // }

    // public Command timingDriveRightSide() {
    //     return new InstantCommand(() -> {timer.restart();})
    //         .andThen(new RunCommand(()->{s_Swerve.driveRight();}, s_Swerve))
    //         .until(() -> {return timer.get() > 1;})
    //         .andThen(new RunCommand(()->{s_Swerve.driveForward();}, s_Swerve))
    //         .until(() -> {return timer.get() > 3;});
    // }
}
