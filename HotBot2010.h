/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.     
 *					                                                          */
/*----------------------------------------------------------------------------*/
#ifndef _HotbotInfo_H_
#define _HotbotInfo_H_
// Customized WPILib Classes
#include "DashboardDataSender.h"
// Team 67 Classes
#include "Navigate.h"
#include "Gamepad.h"
#include "HotPID.h"
// Standard Includes
#include <math.h>
#include "Target.h"
// Local Copy of WPILib
#include "WPILib.h"

// Code include defines
//#define CAMERA_ON	// uncomment to turn on the camera
//#define DASHBOARD_ON // uncomment to turn on the dashboard
#define WATCHDOG_ON // uncomment to turn on the watchdog
//#define PID_TEST // uncomment to turn on test PID controls
// Hardware and software system constants
#define KICKERBOTTOM				3.4		// pot volts
#define POTMAXVALUE					4.3		// pot volts
#define POTMINVALUE					2.1		// pot volts
#define KICKINGTIMEDELAY			2.0
#define ALLOWEDPOTENTIOMETERERROR	0.12		// Reduce this as much as possible!!!
#define STANDMAXVALUE
#define STANDMINVALUE
#define STANDBOTTOM					1.5
#define ALLOWEDPOTENTIOMETERERRORSTAND	0.5		// Reduce this as much as possible!!!
#define LOOPSPERSEC					50

#endif
