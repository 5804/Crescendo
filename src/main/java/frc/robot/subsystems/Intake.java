package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private TalonFX intakeMotor;
    private TalonFX angleMotor;
    private CANcoder angleEncoder;

    public Intake() {
        intakeMotor = new TalonFX(56); // Reset these valuse later
        intakeMotor.setInverted(true);
        //angleMotor = new TalonFX(100);
        angleEncoder = new CANcoder(54);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    double intakeSpeed = 0.6;
    boolean enableIntake = true;
    public Command setIntakeSpeedCommand() {
        return runOnce(
                () -> {
                if (enableIntake == true) {
                    setIntakeSpeed(intakeSpeed); 
                    enableIntake = false;
                } else {
                    setIntakeSpeed(0);
                    enableIntake = true;
                }
                })
            .withName("SetIntakeSpeed");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Rotation2d rotations = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
        SmartDashboard.putNumber("IntakeAngle", rotations.getDegrees());
    }
}

