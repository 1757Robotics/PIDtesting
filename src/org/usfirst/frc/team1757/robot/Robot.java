package org.usfirst.frc.team1757.robot;


import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends SampleRobot {

//AnalogGyro gyrometer;
  CANTalon motor;
  CANTalon gearmotor;
  BuiltInAccelerometer accel;
  ADXL362 newAccel;
  ADXRS450_Gyro newGyro;
  Encoder enc;
  Joystick gamepad;
  PIDController motorpidController;
  PIDController gyropidController;
  PIDSubsystem motorpidSubsystem;
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;
  double Kf = 0;
  
  public Robot() {
	  
	  gamepad = new Joystick(0);    
	  
      motor  = new CANTalon(0); // Initialize the CanTalonSRX on device 1.
      gearmotor = new CANTalon(2); 
      LiveWindow.addActuator("Drive", "motor1", motor);
      enc = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
     
      
      enc.reset();
      enc.setPIDSourceType(PIDSourceType.kDisplacement);
      // Posistion source, not rate 
      
      newGyro = new ADXRS450_Gyro();
      LiveWindow.addSensor("Drive Control", "New Gyro", newGyro);
      
      newAccel = new ADXL362(Accelerometer.Range.k8G);  
      LiveWindow.addSensor("Drive Control", "New Accel", newAccel);
  
      
      motorpidController = new PIDController(0, 0, 0, 0, enc, gearmotor);
      // input source and set point are based on 4096 counts per rotation
      // output source is from -1 to 1 scale for Voltage 
      LiveWindow.addActuator("Drive Control", "PID Controller", motorpidController);
      
      
  }
  
  public void operatorControl() {
	  Long startTime = null;
	  long timeDifference = 0;
	  motorpidController.setPID(0, 0, 0, 0);
	  motor.setEncPosition(0);
	  while(isOperatorControl() && isEnabled()){
		  motor.set(gamepad.getY()*.1);
		  motorpidController.setPID(Kp, Ki, Kd, Kf);
		  if (gamepad.getRawButton(2)== true){
			  motorpidController.setSetpoint(994);
			  startTime = System.currentTimeMillis();
		  }
		  if (gamepad.getRawButton(4) == true){
			  motorpidController.setSetpoint(0);
		  }
		  if (gamepad.getRawButton(1) == true){
			  Kp += .00001;
		  }
		  if (gamepad.getRawButton(3) == true){
			  Kp -= .00001;
		  }
		  if (gamepad.getRawButton(7) == true){
			  Ki += .00001;
		  }
		  if (gamepad.getRawButton(8) == true){
			  Ki -= .00001;
		  }
		  if (gamepad.getPOV(0)== 270){
			  Kd += .00001;
		  }
		  if (gamepad.getPOV(0) == 90){
			  Kd -= .00001;
		  }
		  if (startTime != null && Math.abs(motorpidController.getError()) < 2){
			  timeDifference = System.currentTimeMillis() - startTime;
			  
			  startTime = null;
		  }
		  SmartDashboard.putNumber("error", motorpidController.getError());
		  SmartDashboard.putNumber("Time to zero", timeDifference);
		  SmartDashboard.putNumber("Kp", Kp);
		  SmartDashboard.putNumber("Ki", Ki);
		  SmartDashboard.putNumber("Kd", Kd);
		  SmartDashboard.putNumber("Kf", Kf);
		  SmartDashboard.putNumber("Controller", gamepad.getY()*.4);
		  SmartDashboard.putData("pidController", motorpidController);
		  SmartDashboard.putNumber("Encoder Count", enc.getRaw() );
	      SmartDashboard.putNumber("Encoder Rate", enc.getRate() );
	      
	     
	  }
	  
  }
}