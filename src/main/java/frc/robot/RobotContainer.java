package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        //bButton.onTrue(shooterSubsystem.setShooterSpeedCommand(1));
        //xButton.onTrue(shooterSubsystem.setShooterSpeedCommand(0.3));
        //bButton.toggleOnTrue(shooterSubsystem.enableShooter());
        //bButton.toggleOnFalse(shooterSubsystem.disableShooter());
        //xButton.onTrue(shooterSubsystem.loadCommand());
        //rightBumper.whileTrue(shooterSubsystem.amp());
        //leftBumper.whileTrue(shooterSubsystem.stow());
        //aButton.onTrue(shooterSubsystem.setShooterSpeedCommand(false));

        //aButton.onTrue(intakeSubsystem.setIntakeSpeedCommand());
        //aButton.onTrue(shooterSubsystem.setIndexerSpeedCommand());

        //bButton.whileTrue(intakeSubsystem.raiseIntake());
        //xButton.whileTrue(intakeSubsystem.lowerIntake());
        //aButton.whileTrue(intakeSubsystem.stow());
        //bButton.whileTrue(intakeSubsystem.deploy());
        // aButton.onTrue(LEDSubsystem.SetAnimationFire());

        //xButton.onTrue(shooterSubsystem.setShooterSpeedCommand(1));
        //xButton.onTrue(shooterSubsystem.setIndexerSpeedCommand());
        //xButton.onTrue(intakeSubsystem.setIntakeSpeedCommand());

        //bButton.whileTrue(intakeSubsystem.stow());
        //xButton.whileTrue(intakeSubsystem.deploy());

        // Deploy Shooter and Intake

        // driver.leftMenu.toggleOnTrue(transform());
        driver.back().toggleOnTrue(transform());
        // leftMenu.toggleOnTrue(transform());

        driver.x().onTrue(transform());
        // xButton.onTrue(transform());

        // Stow Shooter and Intake
        driver.a().onTrue(stowParallel());
        //aButton.onTrue(stow());
        
        driver.start().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // rightMenu.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        
        driver.b().onTrue(shooterSubsystem.amp());
        // bButton.onTrue(shooterSubsystem.amp());
        //dDown1.onTrue(shooterSubsystem.stow());
        
        //driver.povLeft().onTrue(LEDSubsystem.lightsOff());
        //driver.povDown().onTrue(shooterSubsystem.lowerShooter());
        //driver.povUp().onTrue(shooterSubsystem.raiseShooter());

        //rightBumper.onTrue(shooterSubsystem.setShooterSpeedCommand(1));
        //driver.leftBumper().onTrue(smartIntake());
        // leftBumper.onTrue(smartIntake());

        // rightTrigger.onTrue()
        // Temp?
        // rightTrigger.onTrue(shooterSubsystem.setIndexerSpeedCommand());

        //driver.rightBumper().onTrue(shooterSubsystem.setShooterSpeedCommand(1));
        // rightBumper.onTrue(shooterSubsystem.setShooterSpeedCommand(1));
        //yButton.onTrue(shooterSubsystem.setIndexerSpeedCommand(0.8));

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
            .until(() -> {return intakeSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.05;})
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
            .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.03;})
            .andThen(new InstantCommand(() -> {shooterSubsystem.deactivateRatchet();}))
            .withName("Climbing");
    }

    public Command smartIntake() {
        return  
            new ParallelCommandGroup(
                shooterSubsystem.setIndexerSpeed(1),
                intakeSubsystem.setIntakeSpeed(0.8),
                new InstantCommand(() -> {shooterSubsystem.setAnglePosition(0.0125);})
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
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return twoNoteAuto();
        return new PathPlannerAuto("twoNoteAuto");
        // return new exampleAuto(s_Swerve);
    }

        // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             12,
    //             0,
    //             0),
    //         s_Swerve.kinematics,
    //         10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Swerve.swerveKinematics);
            // Apply the voltage constraint
            // .addConstraint(autoVoltageConstraint);
    

    // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         // Pass config
    //         config);

    /*RamseteCommand ramseteCommand =
        new RamseteCommand(exampleTrajectory, null, null, null, null, null)
        new RamseteCommand(
            exampleTrajectory,
            s_Swerve::getPose,
            new RamseteController(2.75, 0),
            // new SimpleMotorFeedforward(
            //     DriveConstants.ksVolts,
            //     DriveConstants.kvVoltSecondsPerMeter,
            //     DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.Swerve.swerveKinematics,
            s_Swerve::getModuleStates
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    Reset odometry to the initial pose of the trajectory, run path following
    command, then stop at the end.
    return Commands.runOnce(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()))
        .andThen(ramseteCommand)
        .andThen(Commands.runOnce(() -> m_robotDrive.tankDriveVolts(0, 0)));
        
    }*/

    public Command twoNoteAuto() {
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
                new PathPlannerAuto("2coneAuto"))
                );

    }
}
