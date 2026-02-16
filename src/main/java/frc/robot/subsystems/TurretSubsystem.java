package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

   private final VelocityVoltage flywheel_request = new VelocityVoltage(0).withSlot(0);

   private final double kP = 0.02;
   private double turnPower = 0;

   private NetworkTable photonTable;
  private NetworkTableEntry targetYaw;






    //Vision vision = new Vision();
    public TurretSubsystem() {
        // axis 1: full body rotation
        rotationMotor = new TalonFX(Constants.TurretConstants.kRotationMotorID);
        //rotationMotor.getConfigurator().apply(Constants.TurretConstants.kRotationConfig);
        rotationMotor.setNeutralMode(NeutralModeValue.Coast);
        
        // axis 2: hood
        hoodMotor = new TalonFX(Constants.TurretConstants.kHoodMotorID);
        hoodMotor.getConfigurator().apply(Constants.TurretConstants.kHoodConfig);
        // hoodMotor.setNeutralMode(NeutralModeValue.Coast);

        flywheelMotor = new TalonFX(Constants.TurretConstants.kFlywheelMotorID); //This is supposed to be 
        flywheelMotor.getConfigurator().apply(Constants.TurretConstants.kFlywheelConfig);
        flywheelMotor.setNeutralMode(NeutralModeValue.Coast);
      
        //Reverse direction of the flywheel 
        MotorOutputConfigs flywheelConfigs = new MotorOutputConfigs();
        flywheelConfigs.Inverted=InvertedValue.Clockwise_Positive;
        flywheelMotor.getConfigurator().apply(flywheelConfigs);

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

        // rotationMotor.getConfigurator().apply(slot0Configs);
        hoodMotor.getConfigurator().apply(slot0Configs);

        var slot0ConfigsFlywheel = new Slot0Configs();
        slot0ConfigsFlywheel.kS = 0.1;
        slot0ConfigsFlywheel.kV = 0.12;
        slot0ConfigsFlywheel.kP = 0.11;
        slot0ConfigsFlywheel.kI = 0;
        slot0ConfigsFlywheel.kD = 0;
        

        flywheelMotor.getConfigurator().apply(slot0ConfigsFlywheel);

        // Smart Dashboard Values
        SmartDashboard.putNumber("Hood Position", 0);
        SmartDashboard.putNumber("Hood Angle", 0);
        SmartDashboard.putNumber("Rotational Position", 0);
        SmartDashboard.putNumber("Rotation Angle", 0);

        SmartDashboard.putNumber("turnPower", 0);

        SmartDashboard.putNumber("Shooter Velocity", 0);

        photonTable = NetworkTableInstance.getDefault().getTable("photonvision");
        targetYaw = photonTable.getEntry("turretcamera/targetYaw");

        
    }

    // sets the position of the entire turret
    // public void setTurretPosition(double position) {
    //     rotationMotor.setControl(rot_MMEV.withPosition(position));
    // }

    public void periodic()
    {
        //double yaw = SmartDashboard.getNumber("Yaw", 0);
        //System.out.println(yaw);

        // Display values from turret's rotational position
        double turretPos = rotationMotor.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("Rotational Position", turretPos);
        SmartDashboard.putNumber("Rotation Angle", turretPos);
        // Display Values from Hood Position
        double hoodPos = hoodMotor.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("Hood Position", hoodPos);
        SmartDashboard.putNumber("Hood Angle", hoodPos / TurretConstants.HOOD_ANGLE_RATIO);

        double shooterVelocity = flywheelMotor.getVelocity().getValueAsDouble()*60;
        SmartDashboard.putNumber("Shooter Velocity", shooterVelocity);
        

        

    }


    public void setTurretPosition(double pos) {
        rotationMotor.setControl(m_request.withPosition(pos));
    }

    public void setFlywheelSpeed(double speed) {
        flywheelMotor.set(speed);
    }

    public void setFlywheelVelocity(double velocity){
        flywheelMotor.setControl(flywheel_request.withVelocity(velocity).withFeedForward(0.5));
    }

    public boolean isFlywheelRunning() {
        return flywheelMotor.get() > (TurretConstants.flywheelSpeed-(TurretConstants.flywheelSpeed*0.25));
    }

    // resets encoder
    public void resetRotationEncoder() {
        rotationMotor.setPosition(0);
    }

    // positive value
    public void setTransferMotorSpeed(double speed){
          transferMotor.set(speed);
    }

    public void setHoodAngle(double pos) {
        hoodMotor.setControl(m_request.withPosition(pos * TurretConstants.HOOD_ANGLE_RATIO));
    }

    public void changeHoodAngle(double amt) { // amt is a multiplier
        double currentPos = hoodMotor.getPosition().getValue().magnitude(); // in degrees
        double updatedPos = currentPos + (amt);
        if (updatedPos > 80) updatedPos = 80;
        if (updatedPos < 0) updatedPos = 0;
        setHoodAngle(updatedPos);
    }

    public void setTurretSpeed() { //Aim to Camera
        double yaw = targetYaw.getDouble(0); // Get Yaw
        turnPower = -kP * yaw;
        if (turnPower > 0.3) {
            turnPower = 0.3;
        }
        if (turnPower < -0.3) {
            turnPower = -0.3;
        }
        rotationMotor.set(turnPower);
        SmartDashboard.putNumber("turnPower", turnPower);
        //System.out.println(turnPower);
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

     public Command shootWhileHeldVelocity(double flywheelVelocity, double transferSpeed){
        return Commands.sequence(
            this.runOnce(() -> { setFlywheelVelocity(flywheelVelocity); }),
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

    public Command setHoodAnglePos(double angle) {
        return Commands.run(() -> this.setHoodAngle(angle));
    }

    public Command changeHoodAnglePos(double amt) {
        return Commands.run(() -> this.changeHoodAngle(amt));
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

   public Command aimTurretCommand() {

       return Commands.run(  () -> this.setTurretSpeed());

      //return this.run(() -> this.rotationMotor.set(turnPower));
   }

   public Command stopTurretCommand() {
        return this.run(() -> this.rotationMotor.set(0));
   }

//    public Command autoStartLauncher(){
//       //Starts Launcher
//       return this.runOnce(() -> {
        
//       });
//    }
}