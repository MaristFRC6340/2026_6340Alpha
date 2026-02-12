package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.swervedrive.Vision;

public class TurretSubsystem extends SubsystemBase {

   // TalonFX to 
    TalonFX transferMotor;
    TalonFX	rotationMotor;
    TalonFX	hoodMotor;
    TalonFX	flywheelMotor;
    TalonFX vectorMotor; //How are we defining this?
   
   private final MotionMagicExpoVoltage rot_MMEV = new MotionMagicExpoVoltage(0);

   private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
   


    //Vision vision = new Vision();
    public TurretSubsystem() {
        // axis 1: full body rotation
        // rotationMotor = new TalonFX(Constants.TurretConstants.kRotationMotorID);//CAN Ids fixed once figured out
        // rotationMotor.getConfigurator().apply(Constants.TurretConstants.kRotationConfig);
        // rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // axis 2: hood
        hoodMotor = new TalonFX(Constants.TurretConstants.kHoodMotorID);
        hoodMotor.getConfigurator().apply(Constants.TurretConstants.kHoodConfig);
        // hoodMotor.setNeutralMode(NeutralModeValue.Coast);

        flywheelMotor = new TalonFX(Constants.TurretConstants.kFlywheelMotorID); //This is supposed to be 
        flywheelMotor.getConfigurator().apply(Constants.TurretConstants.kFlywheelConfig);
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        transferMotor = new TalonFX(Constants.TurretConstants.kTransferMotorID);
        transferMotor.getConfigurator().apply(Constants.TurretConstants.kTransferConfig);
        transferMotor.setNeutralMode(NeutralModeValue.Coast);

        // vectorMotor = new TalonFX(Constants.TurretConstants.kVectorMotorID);
        // vectorMotor.getConfigurator().apply(Constants.TurretConstants.kVectorConfig);
        // vectorMotor.setNeutralMode(NeutralModeValue.Coast);
       

        //rotationMotor.setControl(rot_MMEV);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0;

        hoodMotor.getConfigurator().apply(slot0Configs);

        // Smart Dashboard Values
        SmartDashboard.putNumber("Hood Position", 0);
        SmartDashboard.putNumber("Hood Angle", 0);

        
    }

    // sets the position of the entire turret
    // public void setTurretPosition(double position) {
    //     rotationMotor.setControl(rot_MMEV.withPosition(position));
    // }

    public void periodic()
    {
        //double yaw = SmartDashboard.getNumber("Yaw", 0);
        //System.out.println(yaw);

        // Display Values from Hood Position
        double hoodPos = hoodMotor.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("Hood Position", hoodPos);
        SmartDashboard.putNumber("Hood Angle", hoodPos / TurretConstants.HOOD_ANGLE_RATIO);

    }

    public void setFlywheelSpeed(double speed) {
        flywheelMotor.set(speed);
    }

    public boolean isFlywheelRunning() {
        return flywheelMotor.get() > (TurretConstants.flywheelSpeed-(TurretConstants.flywheelSpeed*0.25));
    }

    // resets encoder
//     public void resetRotationEncoder() {
//         rotationMotor.setPosition(0);
//     }

    // positive value
    public void setTransferMotorSpeed(double speed){
          transferMotor.set(speed);
    }

    public void setHoodAngle(double pos) {
        hoodMotor.setControl(m_request.withPosition(pos * TurretConstants.HOOD_ANGLE_RATIO));
    }

    /* COMMANDS */



    public Command getSetFlywheelCommand(double speed) {
        return this.startEnd(() -> {
            setFlywheelSpeed(speed);
        }, () -> {
            setFlywheelSpeed(0);
        });
    }

    public Command getSetTransferCommand(double speed){
        //powers transfer motor on operator input
       return this.startEnd(() -> {
          setTransferMotorSpeed(speed);
       }, () -> { 
         setTransferMotorSpeed(0);
        });
     }
       
     public Command shootWhileHeld(double flywheelSpeed, double transferSpeed){
        return Commands.sequence(
            this.runOnce(() -> { setFlywheelSpeed(flywheelSpeed); }),
            Commands.waitSeconds(1),
            this.run(() -> { setTransferMotorSpeed(transferSpeed); })
        ).finallyDo(interrupted -> {
            setFlywheelSpeed(0);
            setTransferMotorSpeed(0);
        });
     }
    // public Command getSetPositionCommand(double position) {
    //    //sets rotational position of turret
    //     return this.runOnce(() -> {
    //         setTurretPosition(position);
    //     });
    // }

    public Command getSetHoodAngleHigh(){
       //Sets hood angle position on operator input
       return Commands.run(() -> this.setHoodAngle(TurretConstants.NEAR_ANGLE)); // Now by Postive Angle
    }

     public Command getSetHoodAngleLow(){
       //Sets hood angle position on operator input
       return Commands.run(() -> this.setHoodAngle(TurretConstants.ZERO_ANGLE)); // Now by Positive Angle
    }

    

   public Command intakeSpeedCommand(){
      //Sets Intake Speed with Trigger Inputs
      return this.runOnce(() -> {

      });
   }

   public Command stopLauncher(){
      //calls the stop() command
      return this.runOnce(() -> {
        
      });
   }


//    public Command autoStartLauncher(){
//       //Starts Launcher
//       return this.runOnce(() -> {
        
//       });
//    }
}