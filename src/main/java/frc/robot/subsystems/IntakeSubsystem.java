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

    PositionVoltage pos_request = new PositionVoltage(0).withSlot(0);

    double pivotPos = 0;

    public IntakeSubsystem() {
        intakePivot = new TalonFX(IntakeConstants.kPivotMotorID);
        intakeRoller = new TalonFX(IntakeConstants.kRollerMotorID);
        
        // applies set configs for pivot in Constants
        intakePivot.getConfigurator().apply(IntakeConstants.kPivotConfig);

        SmartDashboard.putNumber("Pivot Position", 0);

        intakePivot.setPosition(0.0); // resets pivot encoder
    }

    public void periodic() {
        pivotPos = intakePivot.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("Pivot Position", pivotPos); // measured in rotations
    }

    public void setPivotSpeed(double speed) {
        intakePivot.set(speed);
    }

    // in rotations
    // Do not Use for Now - michaudc
    public void setPivotAngle(double pos){
        intakePivot.setControl(pos_request.withPosition(pos));
        // intakePivot.setControl(test_request.withPosition(pos));
    }

    public void setRollerSpeed(double speed){
        intakeRoller.set(speed);
    }

    // # COMMANDS

    public Command getSetPivotSpeed(double speed) {
        return Commands.startEnd(() -> {
            this.setPivotSpeed(speed);
        }, () -> {
            this.setPivotSpeed(0);
        });
    }

    public Command setAgitationCommand(double amt) {
        return Commands.sequence(
            this.runOnce( () -> {this.setPivotSpeed(amt);} ),
            Commands.waitSeconds(0.1),
            this.runOnce( () -> {this.setPivotSpeed(-amt);} ),
            Commands.waitSeconds(0.1),
            this.runOnce( () -> {this.setPivotSpeed(amt);} ),
            Commands.waitSeconds(0.1),
            this.runOnce( () -> {this.setPivotSpeed(-amt);} ),
            Commands.waitSeconds(0.1)
        ).andThen( () -> {this.setPivotSpeed(0); } );
    }

    // Do Not Use now
    // michaudc
    public Command intakeDownCommand(){
        return Commands.run(() -> this.setPivotAngle(IntakeConstants.INTAKE_DOWN_POS));
    }

    // Do Not use now
    // michaudc
    public Command intakeUpCommand(){
        return Commands.run(() -> this.setPivotAngle(0));
    }

    // Do Not use now
    // michaudc
    public Command setIntakePositionCommand(double pos) {
        return Commands.runOnce(() -> this.setPivotAngle(pos));
    }

    // Use This
    public Command setRollerSpeedCommand(double speed){
        return Commands.startEnd(
            () -> setRollerSpeed(speed),
            () -> setRollerSpeed(0)
        );
    }

}
