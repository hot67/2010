/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#include "Navigate.h"
#include <math.h>
/**
 * The constructor creates the objects used for navigation sensor input and
 * diagnostic output.  Encoder and display objects are created.
 * @param LoopsPerSec The execution rate of the <code>update()</code> method.
 */
Navigate::Navigate(double LoopsPerSec)
{
	
	m_loopRate = LoopsPerSec;
	
	m_dsLCD = DriverStationLCD::GetInstance();
	
    m_useGyro = true;
    
    m_useAccel = false;
    
    m_velocityAlpha = 0.9;
	
	m_leftDrive		= new Encoder(1, 2, true);
	m_leftDrive->SetDistancePerPulse(INCHESPERCLICK);
	m_leftDrive->SetMaxPeriod(1.0);
	m_leftDrive->Start();
	
	m_rightDrive 	= new Encoder(3, 4, false);
	m_rightDrive->SetDistancePerPulse(INCHESPERCLICK);
	m_rightDrive->SetMaxPeriod(1.0);
	m_rightDrive->Start();
	
    if(m_useGyro) {
        m_gyro = new Gyro(1);
        m_gyro->SetSensitivity(GYROSENSITIVITY);
    }
    
    if(m_useAccel) {
        m_accel = new Accelerometer(4);
        m_accel->SetSensitivity(ACCELSENSITIVITY);
    }
}
Navigate::~Navigate(void)
{
}
/**
 * Reset the the input sensors and navigation variables to zero.
 */
void Navigate::Reset(void)
{
	m_leftDrive->Reset();	
	m_rightDrive->Reset();

    if(m_useGyro) {	
    	m_gyro->Reset();
    } else {
    	m_heading = 0.0;
    }
    
    if(m_useAccel) {	
    } else {
        m_headingZ = 0.0;
    }
	
	m_leftDriveDistanceLast = 0.0;
	m_rightDriveDistanceLast = 0.0;

	m_leftDriveVelocity 	= 0.0;
	m_rightDriveVelocity = 0.0;
	
	m_position_x = 0.0;
	m_position_y = 0.0;

 } /* END Reset */

/**
 * Read the current positions of the input sensors and upate the navigation
 * variables to the current position and velocity.
 */
void Navigate::Update(void)
{
	
	m_leftDriveCount = m_leftDrive->Get();
	m_leftDriveDistance = m_leftDrive->GetDistance();
	m_leftDriveRate = m_leftDrive->GetRate();
	
	m_rightDriveCount = m_rightDrive->Get();
	m_rightDriveDistance = m_rightDrive->GetDistance();
	m_rightDriveRate = m_rightDrive->GetRate();
	
    /**
     * Velocity Calculation Method 1
     */
  	// calculate the incremental wheel travel distance since the last call
	double leftDriveDeltaD 	= m_leftDriveDistance - m_leftDriveDistanceLast;
	double rightDriveDeltaD = m_rightDriveDistance - m_rightDriveDistanceLast;
	
    // calclulate velocities using incremental wheel travel distance and expontial smoothing
   	m_leftDriveVelocity     = m_leftDriveVelocity*(1.0 - m_velocityAlpha) + leftDriveDeltaD*m_loopRate*m_velocityAlpha;
   	m_rightDriveVelocity    = m_rightDriveVelocity*(1.0 - m_velocityAlpha) + rightDriveDeltaD*m_loopRate*m_velocityAlpha;

    // save the current wheel travel distance for use the next time around
    m_leftDriveDistanceLast	= m_leftDriveDistance;
    m_rightDriveDistanceLast = m_rightDriveDistance;
	
	// use a gyro to sense the heading
    if (m_useGyro){
    		m_heading = -m_gyro->GetAngle();
    }
    // use encoder measurements to calculate the heading
    else {
    		m_heading += (rightDriveDeltaD - leftDriveDeltaD)/ WHEELTRACK;
    }
    
    // use a accelerometer to get acceleration
        if (m_useAccel){
        		m_headingZ = -m_accel->GetAcceleration();
        }

	/**
     * Calculate the new current x and y coordinates
     * x = straight ahead
     * y = right
     */
	double forwardDeltaD = ( leftDriveDeltaD + rightDriveDeltaD )/2.0;
	m_position_x -= forwardDeltaD*cos((m_heading));
	m_position_y -= forwardDeltaD*sin((m_heading));
	
 } /* END Update */
/**
 * Display the current encoder count on the console (counts).
 */
void Navigate::ShowCountConsole(void)
{

	printf("Encoders: LD=%8ld, RD=%8ld\n\r",
			m_leftDriveCount,m_rightDriveCount);

} /* END ShowCountConsole */
/**
 * Display the current encoder count on the dashboard (counts).
 */
void Navigate::ShowCountDriverStation(void)
{

	m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1,"C:L=%7ld R=%7ld\n\r",
			m_leftDriveCount,m_rightDriveCount);
	m_dsLCD->UpdateLCD();

} /* END ShowCountConsole */
/**
 * Display the current encoder count on the dashboard (counts).
 */
void Navigate::ShowVelocityDriverStation(void)
{

	m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1,"V:L=%8.4f R=%8.4f\n\r",
			m_leftDriveVelocity,m_rightDriveVelocity);
	m_dsLCD->UpdateLCD();
} /* END ShowCountConsole */
/**
 * Display the current encoder count on the dashboard (counts).
 */
void Navigate::ShowDistanceDriverStation(void)
{

	m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1,"D:L=%7.0f R=%7.0f\n\r",
			m_leftDriveDistance,m_rightDriveDistance);
	m_dsLCD->UpdateLCD();

} /* END ShowCountConsole */
/**
 * Display the current encoder count on the dashboard (counts).
 */
void Navigate::ShowPositionDriverStation(void)
{

	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1,"P:X=%7.4f Y=%7.4f\n\r",
			m_position_x,m_position_y);
	m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,"p:h=%7.4f b=%7.4f\n\r",m_heading*180./acos(-1), m_bearing*180./acos(-1));
	m_dsLCD->UpdateLCD();

} /* END ShowCountConsole */
/**
 * Display the current encoder count on the dashboard (counts).
 */
void Navigate::ShowRateDriverStation(void)
{

	m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1,"R:LD=%8.4f, RD=%8.4f\n\r",
			m_leftDriveRate,m_rightDriveRate);
	m_dsLCD->UpdateLCD();

} /* END ShowCountConsole */
/**
 * Display the current encoder count on the dashboard (counts).
 */
void Navigate::ShowCountDashboard(void)
{
//	m_dashboard->Printf("Encoders: LD=%8ld, RD=%8ld\n\r",
//			m_leftDriveCount,m_rightDriveCount);
} /* END ShowCountDashboard */
/**
 * Display the current wheel travel distance on the console (meters).
 */
void Navigate::ShowDistanceConsole(void)
{

	printf("Encoders: LD=%8.3f, RD=%8.3f\n\r",
			m_leftDriveDistance,m_rightDriveDistance);

} /* END ShowDistanceConsole */
/**
 * Display the current wheel travel distance on the dashboard (meters).
 */
void Navigate::ShowDistanceDashboard(void)
{

//	m_dashboard->Printf("Encoders: LD=%8.3f, RD=%8.3f\n\r",
//			m_leftDriveDistance,m_rightDriveDistance);

} /* END ShowDistanceDashboard */
/**
 * Display the current wheel velocity on the console (meters/second).
 */
void Navigate::ShowVelocityConsole(void)
{

	printf("Velocity: LD=%8.4f, RD=%8.4f\n\r",
			m_leftDriveVelocity,m_rightDriveVelocity);

} /* END ShowVelocityConsole */
/**
 * Display the current wheel velocity on the dashboard (meters/second).
 */
void Navigate::ShowVelocityDashboard(void)
{

//	m_dashboard->Printf("Velocity: LD=%8.4f, RD=%8.4f\n\r",
//			m_leftDriveVelocity,m_rightDriveVelocity);

} /* END ShowVelocityDashboard */
/**
 * Display the current encoder rate on the console (counts/second).
 */
void Navigate::ShowRateConsole(void)
{

	printf("Rate: LD=%8.4f, RD=%8.4f\n\r",
			m_leftDriveRate,m_rightDriveRate);

} /* END ShowRateConsole */
/**
 * Display the current encoder rate on the dashboard (counts/second).
 */
void Navigate::ShowRateDashboard(void)
{

//	m_dashboard->Printf("Rate: LD=%8.4f, RD=%8.4f\n\r",
//			m_leftDriveRate,m_rightDriveRate);

} /* END ShowRateDashboard */
/**
 * Display the current encoder rate on the console (counts/second).
 */
void Navigate::ShowPositionConsole(void)
{

	printf("Position: X=%8.4f, Y=%8.4f, a=%8.0f\n\r",
			m_position_x,m_position_y,m_heading*180./acos(-1));

} /* END ShowPositionConsole */
/**
 * Display the current encoder rate on the dashboard (counts/second).
 */
void Navigate::ShowPositionDashboard(void)
{

//	m_dashboard->Printf("Position: X=%8.4f, Y=%8.4f, a=%8.0f\n\r",
//			m_position_x,m_position_y,m_heading*180./acos(-1));

} /* END ShowPositionDashboard */




