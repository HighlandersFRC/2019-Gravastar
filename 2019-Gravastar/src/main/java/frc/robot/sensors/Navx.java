package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import jaci.pathfinder.Pathfinder;


public class Navx {
	private double originalAngle;
	private double originalYaw;
	private AHRS imu;

	public Navx( AHRS navx) {
		imu = navx;
		originalAngle = navx.getAngle();
		originalYaw = navx.getYaw();
	}
	public double currentAngle() {
		return imu.getAngle()-originalAngle;	
	}
	public double currentYaw(){
		return ((imu.getYaw())-originalYaw);
	}
	public double currentReverseYaw(){
		return Pathfinder.boundHalfDegrees((imu.getYaw())-originalYaw+180) ;
	}
	public boolean isMoving() {
		return imu.isMoving();
	}

	public boolean isOn(){
		return imu.isConnected();
	}
	public boolean isMagCalibrated(){
		return imu.isMagnetometerCalibrated();
	}
	public boolean isAutoCalibrating(){
		return imu.isCalibrating();
	}
	public boolean isMagInerference(){
		return imu.isMagneticDisturbance();
	}
	public void softResetAngle(double angle){
		originalAngle = angle;

	}
	public void softResetYaw(double yaw){
		originalYaw = yaw;
	}
}
