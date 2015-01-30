package com.mvrt.mpu6050;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MPU_6050 extends SensorBase {
	private final byte ADDRESS = 0x68;
	private final byte PWR_MGMT_1 = 0x6B;
	private final byte WHO_AM_I_ADDR = 0x75;
	private final byte GYRO_RANGE_ADDR = 0x1B;
	private final byte GYRO_RANGE_2000 = 0x18, GYRO_RANGE_1000 = 0x10, GYRO_RANGE_500 = 0x08, GYRO_RANGE_250 = 0x00;
	private final byte ACCEL_RANGE_ADDR = 0x1C;
	private final byte ACCEL_RANGE_16G = 0x18, ACCEL_RANGE_8G = 0x10, ACCEL_RANGE_4G = 0x08, ACCEL_RANGE_2G = 0x00;
	
	private I2C mpu;
	public MPU_6050(GyroRange gyroRange, AccelRange accelRange) {
		mpu = new I2C(I2C.Port.kOnboard, ADDRESS);
		initialize(gyroRange, accelRange);
		
		SmartDashboard.putBoolean("MPU Detected", getDeviceID() == 0x34);
	}
	
	public MPU_6050() {
		this(GyroRange.k2000, AccelRange.k16g);
	}
	
	private void initialize(GyroRange gyroRange, AccelRange accelRange) {
		setGyroRange(gyroRange);
		setAccelRange(accelRange);
		mpu.write(PWR_MGMT_1, 0x01); // clear sleep bit, set gyro clock source to X
	}
	
	public void setGyroRange(GyroRange range) {
		byte GYRO_RANGE = GYRO_RANGE_2000;
		switch (range) {
		case k2000:
			GYRO_RANGE = GYRO_RANGE_2000;
			break;
		case k1000:
			GYRO_RANGE = GYRO_RANGE_1000;
			break;
		case k500:
			GYRO_RANGE = GYRO_RANGE_500;
			break;
		case k250:
			GYRO_RANGE = GYRO_RANGE_250;
			break;
		}
		mpu.write(GYRO_RANGE_ADDR, GYRO_RANGE); // set gyro range
	}
	
	public void setAccelRange(AccelRange range) {
		byte ACCEL_RANGE = ACCEL_RANGE_16G;
		switch(range) {
		case k2g:
			ACCEL_RANGE = ACCEL_RANGE_2G;
			break;
		case k4g:
			ACCEL_RANGE = ACCEL_RANGE_4G;
			break;
		case k8g:
			ACCEL_RANGE = ACCEL_RANGE_8G;
			break;
		case k16g:
			ACCEL_RANGE = ACCEL_RANGE_16G;
			break;
		}
		mpu.write(ACCEL_RANGE_ADDR, ACCEL_RANGE);
	}
	
	private int getDeviceID() {
		byte[] buffer = new byte[1];
		mpu.read(WHO_AM_I_ADDR, 1, buffer);
		return buffer[0];
	}
	
	public enum GyroRange {
		k2000, k1000, k500, k250;
	}
	
	public enum AccelRange {
		k2g, k4g, k8g, k16g;
	}
}
