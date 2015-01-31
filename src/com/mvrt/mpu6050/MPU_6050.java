package com.mvrt.mpu6050;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MPU_6050 extends SensorBase {
	private final byte ADDRESS = 0x68;
	private final byte PWR_MGMT_1 = 0x6B;
	private final byte WHO_AM_I_ADDR = 0x75;
	private final byte GYRO_RANGE_ADDR = 0x1B;
	private final byte GYRO_RANGE_2000 = 0x18, GYRO_RANGE_1000 = 0x10, GYRO_RANGE_500 = 0x08, GYRO_RANGE_250 = 0x00;
	private final byte ACCEL_RANGE_ADDR = 0x1C;
	private final byte ACCEL_RANGE_16G = 0x18, ACCEL_RANGE_8G = 0x10, ACCEL_RANGE_4G = 0x08, ACCEL_RANGE_2G = 0x00;
	private final byte GYRO_DATA_ADDR = 0x43;
	private final byte ACCEL_DATA_ADDR = 0x3B;
	private final byte RESET_ADDR = 0x68;
	private double LSBPerDegrees;
	private int LSBPerGrams;
	
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
		setAccelRange(accelRange);
		mpu.write(PWR_MGMT_1, 0x01); // clear sleep bit, set gyro clock source to X
		mpu.write(0x19, 0x4F); // set sample rate to 100Hz (8000 / (1 + 0x4F))
	}
	
	public void setGyroRange(GyroRange range) {
		byte GYRO_RANGE = GYRO_RANGE_2000;
		switch (range) {
		case k2000:
			GYRO_RANGE = GYRO_RANGE_2000;
			LSBPerDegrees = 16.4;
			break;
		case k1000:
			GYRO_RANGE = GYRO_RANGE_1000;
			LSBPerDegrees = 32.8;
			break;
		case k500:
			GYRO_RANGE = GYRO_RANGE_500;
			LSBPerDegrees = 65.5;
			break;
		case k250:
			GYRO_RANGE = GYRO_RANGE_250;
			LSBPerDegrees = 131;
			break;
		}
		mpu.write(GYRO_RANGE_ADDR, GYRO_RANGE); // set gyro range
	}
	
	public void setAccelRange(AccelRange range) {
		byte ACCEL_RANGE = ACCEL_RANGE_16G;
		switch(range) {
		case k2g:
			ACCEL_RANGE = ACCEL_RANGE_2G;
			LSBPerGrams = 16384;
			break;
		case k4g:
			ACCEL_RANGE = ACCEL_RANGE_4G;
			LSBPerGrams = 8192;
			break;
		case k8g:
			ACCEL_RANGE = ACCEL_RANGE_8G;
			LSBPerGrams = 4096;
			break;
		case k16g:
			ACCEL_RANGE = ACCEL_RANGE_16G;
			LSBPerGrams = 2048;
			break;
		}
		mpu.write(ACCEL_RANGE_ADDR, ACCEL_RANGE);
	}
	
	private int getDeviceID() {
		byte[] buffer = new byte[1];
		mpu.read(WHO_AM_I_ADDR, 1, buffer);
		return buffer[0];
	}
	
	public static class Axes {

		/**
		 * The integer value representing this enumeration
		 */
		public final byte value;
		static final byte kX_val = 0x00;
		static final byte kY_val = 0x02;
		static final byte kZ_val = 0x04;
		public static final Axes kX = new Axes(kX_val);
		public static final Axes kY = new Axes(kY_val);
		public static final Axes kZ = new Axes(kZ_val);

		private Axes(byte value) {
			this.value = value;
		}
	}
	
	public static class AllAxes {

		public double XAxis;
		public double YAxis;
		public double ZAxis;
	}
	
	public double getXRate() {
		return getRate(Axes.kX);
	}

	public double getYRate() {
		return getRate(Axes.kY);
	}

	public double getZRate() {
		return getRate(Axes.kZ);
	}
	
	public double getRate(Axes axis) {
		byte[] raw = new byte[2];
		mpu.read(GYRO_DATA_ADDR + axis.value, 2, raw);
		
		return ((raw[0] << 8 | raw[1]) / LSBPerDegrees);
	}
	
	public AllAxes getAllRateAxes() {
		byte[] raw = new byte[6];
		mpu.read(GYRO_DATA_ADDR, 6, raw);
		
		AllAxes data = new AllAxes();
		
		data.XAxis = ((raw[0] << 8 | raw[1]) / LSBPerDegrees);
		data.YAxis = ((raw[2] << 8 | raw[3]) / LSBPerDegrees);
		data.ZAxis = ((raw[4] << 8 | raw[5]) / LSBPerDegrees);
		
		return data;
	}
	
	public double getXAcceleration() {
		return getRate(Axes.kX);
	}

	public double getYAcceleration() {
		return getRate(Axes.kY);
	}

	public double getZAcceleration() {
		return getRate(Axes.kZ);
	}
	
	public double getAcceleration(Axes axis) {
		byte[] raw = new byte[2];
		mpu.read(ACCEL_DATA_ADDR + axis.value, 2, raw);
		
		return ((raw[0] << 8 | raw[1]) / LSBPerGrams);
	}
	
	public AllAxes getAllAccelerationAxes() {
		byte[] raw = new byte[6];
		mpu.read(ACCEL_DATA_ADDR, 6, raw);
		
		AllAxes data = new AllAxes();
		
		data.XAxis = ((raw[0] << 8 | raw[1]) / LSBPerGrams);
		data.YAxis = ((raw[2] << 8 | raw[3]) / LSBPerGrams);
		data.ZAxis = ((raw[4] << 8 | raw[5]) / LSBPerGrams);
		
		return data;
	}
	
	public void reset() {
		mpu.write(RESET_ADDR, 0x07);
	}
	
	public enum GyroRange {
		k2000, k1000, k500, k250;
	}
	
	public enum AccelRange {
		k2g, k4g, k8g, k16g;
	}
}
