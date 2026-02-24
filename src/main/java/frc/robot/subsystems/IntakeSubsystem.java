package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    TalonFX intakePivot; // doesn't require PID
    TalonFX intakeRoller;

    PositionVoltage pos_request = new PositionVoltage(0).withSlot(0).withVelocity(5.0);

    double pivotPos = 0;

    public IntakeSubsystem() {
        intakePivot = new TalonFX(IntakeConstants.kPivotMotorID);
        intakeRoller = new TalonFX(IntakeConstants.kRollerMotorID);
        
        // applies set configs for pivot in Constants
        intakePivot.getConfigurator().apply(IntakeConstants.kPivotConfig);

        SmartDashboard.putNumber("Pivot Position", 0);
    }

    public void periodic() {
        pivotPos = intakePivot.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("Pivot Position", pivotPos); // measured in rotations
    }

    public void setPivotSpeed(double speed) {
        intakePivot.set(speed);
    }

    // in rotations
    public void setPivotAngle(double pos){
        intakePivot.setControl(pos_request.withPosition(pos));
    }

    public void setRollerSpeed(double speed){
        intakeRoller.set(speed);
    }

    // # COMMANDS

    public Command getSetPivotSpeed(double speed) {
        return this.startEnd(() -> {
            this.setPivotSpeed(speed);
        }, () -> {
            this.setPivotSpeed(0);
        });
    }

    public Command intakeDownCommand(){
        return Commands.run(() -> this.setPivotAngle(-5)); // down is -11.559570
    }

    public Command intakeUpCommand(){
        return Commands.run(() -> this.setPivotAngle(0));
    }

    public Command setRollerSpeedCommand(double speed){
        return Commands.startEnd(
            () -> setRollerSpeed(speed),
            () -> setRollerSpeed(0)
        );
    }
}
