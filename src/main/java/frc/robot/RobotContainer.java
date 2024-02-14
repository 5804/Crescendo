package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
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
    private final Joystick driver = new Joystick(0);
    private final Joystick buttonBoard = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final JoystickButton rightTrigger = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
    private final JoystickButton leftTrigger = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton increaseShooterSpeedButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton decreaseShooterSpeedButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final POVButton dUp1 = new POVButton(driver, 0);
    private final POVButton dRight1 = new POVButton(driver, 90);
    private final POVButton dDown1 = new POVButton(driver, 180);
    private final POVButton dLeft1 = new POVButton(driver, 270);

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


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        bButton.onTrue(shooterSubsystem.setShooterSpeedCommand());
        //bButton.toggleOnTrue(shooterSubsystem.enableShooter());
        //bButton.toggleOnFalse(shooterSubsystem.disableShooter());
        xButton.onTrue(shooterSubsystem.loadCommand());
        //increaseShooterSpeedButton.onTrue(shooterSubsystem.increaseShooterSpeed());
        //decreaseShooterSpeedButton.onTrue(shooterSubsystem.decreaseShooterSpeed());
        //aButton.onTrue(shooterSubsystem.setShooterSpeedCommand(false));

        aButton.onTrue(intakeSubsystem.setIntakeSpeedCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto("New New New Auto");
        //return new exampleAuto(s_Swerve);
    }
}
