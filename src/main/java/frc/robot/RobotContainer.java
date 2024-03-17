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

    // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    3,
                    2)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.Swerve.swerveKinematics);
                // Apply the voltage constraint
                // .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow. All units in meters.
        Trajectory testTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0.1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, 0, new Rotation2d(90)),
                // Pass config
                config);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("transform", transform());
        NamedCommands.registerCommand("intake", smartIntake());
        NamedCommands.registerCommand("shoot", smartShoot());
        NamedCommands.registerCommand("startShoot", startShoot());
        NamedCommands.registerCommand("thirdNoteShoot", smartShootThirdNote());
        NamedCommands.registerCommand("lastNoteShoot", smartShootLastNote());
        NamedCommands.registerCommand("positionShooter", shooterSubsystem.shootFromNotePosition());
        NamedCommands.registerCommand("shootNote", autoShooter());
        NamedCommands.registerCommand("indexNote", autoIndexer());
        NamedCommands.registerCommand("stopShooter", stopShooter());
        NamedCommands.registerCommand("stopIndexer", stopIndexer());

        NamedCommands.registerCommand("index", indexWithTOF());

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

        // shooterSubsystem.setDefaultCommand(
        //     indexWithTOF()
        // );
        
        // Configure the button bindings
        configureButtonBindings();

        chooser.setDefaultOption("One Note Auto", oneNoteAuto());

        // chooser.addOption("Turn Test",
        //     new PathPlannerAuto("rotateAuto"));

        chooser.addOption("2 Piece Auto Test", twoNoteAuto()
        .finallyDo(() -> {s_Swerve.zeroHeading();})
        );

        // chooser.addOption("3 Piece Auto", threeNoteAuto()
        //     .finallyDo(() -> {s_Swerve.zeroHeading();})
        //     );
        chooser.addOption("4 Piece Auto", fourNoteAuto()
        .finallyDo(() -> {s_Swerve.zeroHeading();})
        );
        // chooser.addOption("4 Piece Auto Test", newFourPieceAuto()
        // .finallyDo(() -> {s_Swerve.zeroHeading();})
        // );
        // chooser.addOption("DO NOT USE YET", midfieldAuto()
        // .finallyDo(() -> {s_Swerve.zeroHeading();})
        // // );
        // chooser.addOption("Midfield Auto", midfield()
        // .finallyDo(() -> {s_Swerve.zeroHeading();})
        // );
        // chooser.addOption("Midfield Auto Test", newMidfieldAuto()
        // .finallyDo(() -> {s_Swerve.zeroHeading();})
        // );

        SmartDashboard.putData("Auto choices", chooser);
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

        driver.b().onTrue(shooterSubsystem.amp());
        
        driver.rightTrigger(0.5).whileTrue(
            shooterSubsystem.setShooterSpeed(1)
            // .until(() -> {return shooterSubsystem.leftShooterMotor.getVelocity().getValue() > 95;})
            .andThen(new ParallelCommandGroup(shooterSubsystem.setIndexerSpeedNoFinallyDo(.8), LEDSubsystem.setRainbowCommand()))
            .finallyDo(() -> {
                shooterSubsystem.load(0);
                shooterSubsystem.shoot(0);
            })
            );
            
        driver.leftTrigger(.2).whileTrue(smartIntake());

        // driver.leftBumper().whileTrue(new InstantCommand(() -> {shooterSubsystem.activateRatchet();}));
        // driver.leftBumper().onTrue(shooterSubsystem.activateRatchet());
        driver.rightBumper().whileTrue(smartClimb());
        driver.rightBumper().onTrue(LEDSubsystem.setRainbowCommand());

        driver.back().onTrue(shooterSubsystem.shootFromNotePosition());

        driver.povUp().whileTrue(shooterSubsystem.increaseShooterPos());
        driver.povDown().whileTrue(shooterSubsystem.decreaseShooterPos());

        b1.onTrue(shooterSubsystem.climbPosition());
        b5.whileTrue(smartOuttake());

        // driver.povRight().whileTrue(intakeSubsystem.raiseIntake());
        // driver.povLeft().whileTrue(intakeSubsystem.lowerIntake());

    }

    public Command transform() {
        return shooterSubsystem.deploy()
            .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() > 0.125;}) //0.13
            .andThen(intakeSubsystem.deploy())
            .until(() -> {return intakeSubsystem.angleEncoder.getAbsolutePosition().getValue() > 0.33;})
            .andThen(shooterSubsystem.stow())
            .withName("OUT (TRANSFORM)");
            // .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() < 0.05;});
    }

    // Deprecated
    public Command stow() {
        return shooterSubsystem.deploy()
            .until(() -> {return shooterSubsystem.angleEncoder.getAbsolutePosition().getValue() > 0.13;}) 
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
            (shooterSubsystem.setShooterSpeedCommand(0.0))
                .andThen(
                new ParallelCommandGroup(
                (shooterSubsystem.setIndexerSpeed(1)),
                intakeSubsystem.setIntakeSpeed(0.8),
                new InstantCommand(() -> {shooterSubsystem.setAnglePosition(0.0225);}) // 0.0125
                ))
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
        }

    public Command smartOuttake() {
        return  
            new ParallelCommandGroup(
                shooterSubsystem.setIndexerSpeed(-1),
                intakeSubsystem.setIntakeSpeed(-0.8),
                new InstantCommand(() -> {shooterSubsystem.setAnglePosition(0.0225);}) // 0.0125
                )
            .andThen(new InstantCommand(() -> {shooterSubsystem.load(0);}))
            .andThen(new InstantCommand(() -> {intakeSubsystem.load(0);}))
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
        }

    public Command autoShooter() {
            return 
            shooterSubsystem.setShooterSpeed(1);
        }

    public Command autoIndexer() {
            return
            shooterSubsystem.setIndexerSpeed(0.8)
            .until(() -> {return shooterSubsystem.TOF.getRange() > 400;});
    }

    public Command stopShooter() {
            return
            shooterSubsystem.setShooterSpeedCommand(0.0);
    }        

    public Command stopIndexer() {
            return
            shooterSubsystem.setIndexerSpeedCommand(0.0);
    }

    public Command smartShoot() {
            return
            // indexWithTOF()
            (shooterSubsystem.setShooterSpeed(1))
            .andThen((shooterSubsystem.shootFromNotePosition()))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8));
            // .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            // .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            // .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));
        }
    
    public Command startShoot() {
            return
            shooterSubsystem.setShooterSpeed(1)
            .andThen(shooterSubsystem.setIndexerSpeed(0.8));
            // .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            // .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            // .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));
        }

        public Command smartShootThirdNote() {
            return
            (shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.autoThirdNotePosition())
            .andThen(shooterSubsystem.setIndexerSpeed(0.8));
            // .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            // .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            // .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));
        }

        public Command smartShootLastNote() {
            return
            (shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.autoLastNotePosition())
            .andThen(shooterSubsystem.setIndexerSpeed(0.8));
            // .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            // .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            // .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));
        }


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

    public Command followTrajectory(Trajectory trajectory) {
        ProfiledPIDController thetaController = new ProfiledPIDController(15, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicDriveController controller = new HolonomicDriveController(
                new PIDController(1.125, 0, 0.1), 
                new PIDController(1.125, 0, 0.1), 
                thetaController
            );

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(trajectory, s_Swerve::getPose, Constants.Swerve.swerveKinematics, controller, s_Swerve::setModuleStates, s_Swerve);
                
        return Commands.runOnce(
            () -> s_Swerve.setPose(trajectory.getInitialPose()))
            .andThen(swerveControllerCommand)
            .andThen(Commands.runOnce(() -> s_Swerve.drive(new Translation2d(0,0), 0, false, false)));
    }
        

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

    }

    public Command twoNoteAuto() {
        return new InstantCommand(() -> {timer.restart();})
            .andThen(shooterSubsystem.setShooterSpeed(1))
            // .until(() -> {return timer.get() > 1;})
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            // .until(() -> {return timer.get() > 2;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0))
            .andThen(transform())
            // .until(() -> {return timer.get() > 4;})
            .andThen(
                new PathPlannerAuto("twoNoteAuto")
                )
            // STOP MOVING
            .andThen(new InstantCommand(() -> {s_Swerve.drive(new Translation2d(0,0), 0, false, false);}))
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8));
            // .until(() -> {return timer.get() > 10;})
            // .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            // .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));
            // .until(() -> {return timer.get() > 10;})
            // .andThen(shooterSubsystem.setShooterSpeed(1))
            // .until(() -> {return timer.get() > 11;})
            // .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            // .until(() -> {return timer.get() > 12;})
            // .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            // .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));

    }

    public Command threeNoteAuto() {
        return
            (shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0))
            .andThen(transform())
            .andThen(new PathPlannerAuto("threeNoteAuto"))
            .andThen(shooterSubsystem.shootFromNotePosition())
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            
            // STOP MOVING
            .andThen(new InstantCommand(() -> {s_Swerve.drive(new Translation2d(0,0), 0, false, false);}))
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));

    }

    public Command fourNoteAuto() {
        return
            (shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0))
            .andThen(transform())
            .andThen(new PathPlannerAuto("fourNoteAuto"))
            .andThen(shooterSubsystem.autoLastNotePosition())
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            
            // STOP MOVING
            .andThen(new InstantCommand(() -> {s_Swerve.drive(new Translation2d(0,0), 0, false, false);}))
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));

    }

    public Command midfieldAuto() {
        return
            (shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0))
            .andThen(transform())
            .andThen(new PathPlannerAuto("4NoteMidLoadSideAuto"))
            .andThen(shooterSubsystem.autoLastNotePosition())
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            
            // STOP MOVING
            .andThen(new InstantCommand(() -> {s_Swerve.drive(new Translation2d(0,0), 0, false, false);}))
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));

    }

    public Command midfield() {
        return
            (shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .until(() -> {return shooterSubsystem.TOF.getRange() > 400;})
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0))
            .andThen(transform())
            .andThen(new PathPlannerAuto("midfield1Auto"))
            .andThen(shooterSubsystem.autoLastNotePosition())
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .andThen(new PathPlannerAuto("midfield2Auto"))
            .andThen(shooterSubsystem.autoLastNotePosition())
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            .andThen(new PathPlannerAuto("midfield3Auto"))
            .andThen(shooterSubsystem.autoLastNotePosition())
            .andThen(shooterSubsystem.setShooterSpeed(1))
            .andThen(shooterSubsystem.setIndexerSpeed(0.8))
            
            // STOP MOVING
            .andThen(new InstantCommand(() -> {s_Swerve.drive(new Translation2d(0,0), 0, false, false);}))
            .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
            .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));

    }

    public Command newMidfieldAuto() {
        return new PathPlannerAuto("midfieldLoadSideAuto")
        .andThen(new InstantCommand(() -> {s_Swerve.drive(new Translation2d(0,0), 0, false, false);}))
        .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
        .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));
    }

    public Command newFourPieceAuto() {
        return new PathPlannerAuto("fourNoteCompiled")
        .andThen(new InstantCommand(() -> {s_Swerve.drive(new Translation2d(0,0), 0, false, false);}))
        .andThen(shooterSubsystem.setShooterSpeedCommand(0.0))
        .andThen(shooterSubsystem.setIndexerSpeedCommand(0.0));
    }

}
