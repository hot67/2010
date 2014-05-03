/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef _Navigate_H_
#define _Navigate_H_
#include "DriverStationLCD.h"
#include "WPILib.h"
#include "DashboardDataSender.h"

#define PI							3.14159265
#define WHEELDIAMETER				5.13		// inches
#define WHEELTRACK					22.626	//inches
//#define GYROSENSITIVITYDEGREE		0.007	// volts/degree
//#define GYROSENSITIVITY				0.401	// volts/radian
#define GYROSENSITIVITYDEGREE		0.0033	// volts/degree
#define GYROSENSITIVITY				0.189	// volts/radian
#define ACCELSENSITIVITY			1.0     // somethings/something
#define ENCODERCOUNTSPERREV			360
#define CLICKSPERINCH				(ENCODERCOUNTSPERREV)/(PI*WHEELDIAMETER)  // inches
#define INCHESPERCLICK				(PI*WHEELDIAMETER)/(ENCODERCOUNTSPERREV)
/**
 * Implements the Team 67 2009 Hotbot navigation infrastructure.  After creating
 * a navigation object and providing the rate at which navigation will update,
 * the <code>update()</code> method is used to periodically gather sensor data
 * and calculate vehicle position, heading and velocity.
 * <p>
 * <b>Digital Sidecar:</b><br>
 * 	Input 1	- Left Drive Encoder A<br>
 * 	Input 2	- Left Drive Encoder B<br>
 * 	Input 3	- Right Drive Encoder A<br>
 * 	Input 4	- Right Drive Encoder B<br>

 * <p>
 * <b>Analog Module:</b><br>
 * 	Input 1	- Gyro (optional)<br>
 * @author Dave Doerr, Paul Doerr
 */
class Navigate {
public:
	Gyro	*m_gyro;
	bool	m_useGyro;
	Accelerometer *m_accel;	
	bool m_useAccel;
	Encoder	*m_leftDrive;	// left-side drive wheel encoder
	Encoder	*m_rightDrive;	// right-side drive wheel encoder
    /**
     * Velocity of the left-side drive wheel derived from an
     * Andy-Mark-Toughbox-mounted US Digital encoder (meters/second).
     * Used for traction control in class <code>Hotbot2009</code>
     */
	double	m_leftDriveVelocity;		// left drive encoder velocity (i/s)
    /**
     * Velocity of the right-side drive wheel derived from an
     * Andy-Mark-Toughbox-mounted US Digital encoder (meters/second).
     * Used for traction control in class <code>Hotbot2009</code>
     */
	double	m_rightDriveVelocity;		// right drive encoder velocity (i/s)
    /**
     * Velocity of the left-side idler wheel derived from a
     * large-VEX-omniwheel-mounted US Digital encoder (inches/second).
     * Used for traction control in class <code>Hotbot2009</code>
     */
    double   m_position_x;
    /**
     * Y-coordinate of the current vehicle position in inches (actual position of the robot).
     */
    double   m_position_y;
    /**
     * Current heading of the vehicle in radians (actual direction of the robot).
     */
    double   m_heading;
    /**
     * Current magnitude of normal force on the vehicle in m/s^2.
     */
    double   m_headingZ;
    /**
     * X-coordinate of the vehicle destination in inches (desired position of the robot).
     */
    double   m_destination_x;
    /**
     * Y-coordinate of the vehicle destination in inches (desired position of the robot).
     */
    double   m_destination_y;
    /**
     * Bearing of the vehicle in radians (desired direction of the robot).
     */
    double   m_bearing;

	Navigate(double);
	~Navigate(void);

	void Update(void);
	void Reset(void);
	void ShowCountConsole(void);
	void ShowCountDriverStation(void);
	void ShowDistanceDriverStation(void);
	void ShowVelocityDriverStation(void);
	void ShowPositionDriverStation(void);
	void ShowRateDriverStation(void);
	void ShowCountDashboard(void);
	void ShowDistanceConsole(void);
	void ShowDistanceDashboard(void);
	void ShowVelocityConsole(void);
	void ShowVelocityDashboard(void);
	void ShowRateConsole(void);
	void ShowRateDashboard(void);
	void ShowPositionConsole(void);
	void ShowPositionDashboard(void);
    
private:
//  static const double kGyroSensitivity = 0.007;	// 2009 KOP, volts/degree/second
//	static const double kGyroSensitivity = 0.401;	// 2009 KOP, volts/radian/second
//	static const double kGyroSensitivity = 0.0125	// 2006-2008 KOP, volts/degree/second
//	static const double kGyroSensitivity = 0.716	// 2006-2008 KOP, volts/radian/second
//	static const double kGyroSensitivity = 0.0125	// ADXRS150, volts/degree/second
//	static const double kGyroSensitivity = 0.716	// ADXRS150, volts/radian/second
//	static const double kGyroSensitivity = 0.005	// ADXRS300, volts/degree/second
//	static const double kGyroSensitivity = 0.286	// ADXRS300, volts/radian/second
	
	double m_loopRate;				// loop rate for velocity calculation
	double	m_velocityAlpha;		// velocity exponential smoothing constant
		
	long	m_leftDriveCount;			// left drive encoder position (counts)
	long	m_rightDriveCount;			// right drive encoder position (counts)

	double	m_leftDriveDistance;		// left drive wheel distance (i)
	double	m_rightDriveDistance;		// right drive wheel distance (i)
	
	double	m_leftDriveDistanceLast;	// left drive wheel distance (i)
	double	m_rightDriveDistanceLast;	// right drive wheel distance (i)
	
	double	m_leftDriveRate;
	double	m_rightDriveRate;
	
	DriverStation *m_ds;		// Driver Station
    DriverStationLCD *m_dsLCD;	// Driver Station LCD
    DashboardDataSender *m_dds;		// Dashboard

};

#endif
