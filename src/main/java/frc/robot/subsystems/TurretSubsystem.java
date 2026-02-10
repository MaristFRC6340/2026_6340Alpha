package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.Vision;

public class TurretSubsystem extends SubsystemBase {

   // TalonFX	transferMotor = new 
    TalonFX	rotationMotor;
   // TalonFX	hoodMotor = 
    TalonFX	flywheelMotor;
   // TalonFX	vectorMotors = 
   private final MotionMagicExpoVoltage rot_MMEV = new MotionMagicExpoVoltage(0);

    //Vision vision = new Vision();
    public TurretSubsystem() {
        rotationMotor = new TalonFX(Constants.TurretConstants.kRotationMotorID);
        rotationMotor.getConfigurator().apply(Constants.TurretConstants.kRotationConfig);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);

        flywheelMotor = new TalonFX(Constants.TurretConstants.kRotationMotorID);
        flywheelMotor.getConfigurator().apply(Constants.TurretConstants.kRotationConfig);
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        rotationMotor.setControl(rot_MMEV);
    }

    // sets the position of the entire turret
    public void setTurretPosition(double position) {
        rotationMotor.setControl(rot_MMEV.withPosition(position));
    }

    public void setFlywheelSpeed(double speed) {
        flywheelMotor.set(speed);
    }

    public void stopFlywheel() {
        flywheelMotor.set(0);
    }

    // resets encoder
    public void resetRotationEncoder() {
        rotationMotor.setPosition(0);
    }

    /* COMMANDS */
    public Command getSetPositionCommand(double position) {
        return this.runOnce(() -> {
            setTurretPosition(position);
        });
    }

    public Command shootCommand() {
        return this.startEnd(() -> {
            setFlywheelSpeed(TurretConstants.flywheelSpeed);
        }, () -> {
            stopFlywheel();
        });
    }
}
