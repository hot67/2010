/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#ifndef _HotPID_H_
#define _HotPID_H_
/**
 * Implements the Team 67 2009 Hotbot PID control.  This class is intended to be
 * used in the autonomous periodic or teleop periodic loops.
 * @author Adapted from WPILib by Dave Doerr
 */
class HotPID
{
private:
	float m_Kp;			// proportional gain
	float m_Ki;			// integral gain
	float m_Kd;			// derivative gain
	float m_SP_max;		// limits for Set Point
	float m_SP_min;
	float m_PV_max;		// limits for Process Variable
	float m_PV_min;
	float m_MV_max;		// limits for Manipulated Variable
	float m_MV_min;
	float m_error_p;		// last p error term
	float m_error_i;		// last i error term
	float m_error_d;		// last d error term
	float m_tolerance;		// on-target tolerance: fraction of m_SP_max, m_SP_min
	float m_enabled;	// enable/disable control
		
public:
	HotPID(float Kp = 0.0, float Ki = 0.0, float Kd = 0.0, float tolerance = 0.0);
	~HotPID(void);

	float GetMV(float SP, float PV);
	
	void SetGains(float Kp, float Ki, float Kd);

	void SetMVLimits(float MV_max, float MV_min);
	void SetSPLimits(float SP_max, float SP_min);
	void SetPVLimits(float PV_max, float PV_min);

	void SetTolerance(float percent);
	bool OnTarget(void);

	void Enable(void);
	void Disable(void);

	void Reset(void);
	void Print(void);
};

#endif
