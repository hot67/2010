#include "HotBot2010.h"

/**
 * This is the 2010 HotBot code 
 */
class HotBot2010 : public IterativeRobot
{
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;		// robot will use PWM 1-4 for drive motors
	
	// Declare variables for the Speed Controllers
	Jaguar *m_lDrive1;
	Jaguar *m_rDrive1;
	Jaguar *m_lDrive2;
	Jaguar *m_rDrive2;

	Jaguar *m_kickStand1;
	Jaguar *m_kickStand2;
	
	Servo *m_lift1;
	
	Relay *m_kickStand3;
	Relay *m_shifter;
	Relay *m_roller;
	
	// Analog Inputs
	AnalogChannel *m_kickPot;
	AnalogChannel *m_standPot;

#ifdef PID_TEST
	//PID testers
	 double testP; 
	 double testI;
	 double testD;
#endif
	
	// Controllers
	HotPID *m_kickerPosition;
	HotPID *m_kickerPositionReverse;
	HotPID *m_standerPosition;
    HotPID *m_robotDistance;
    HotPID *m_robotTurnStabilize;
    HotPID *m_robotTurnBearing;
	
	Navigate *m_navigate;
	
	// Denotes whether the robot is set for a shot (1) or resetting (0)
	bool m_resetFlag;
	// resetflag for standing
	bool m_standResetFlag;
	// Denotes whether the robot is kicking (1) or not
	bool m_kickingFlag;
	// Tells if it's in manual kicker control or not
	bool m_joystickFlag;
	// Tells if the shifter is in auto or manual mode
	bool m_potFailure;
	// You must take your finger off of the trigger before you can fire again
	bool m_repeatedKick;
	
	bool m_servoed;
	
	// Stores the current desired potentiometer value for shot distance - one of the three above
	double m_shotPosition;
	
	double m_standPosition;
	
	double m_hangingInstantPot;
	
	bool m_instantPotSet;
	
	int m_autonomousCase;				// The current state of the autonomous program.
	int m_autoProgramNumber;			// Autonomous Program currently selected
	int m_startColumnNo;				// variable to select  column
	double m_startColumn;				// column (Y coord) of robot start position
	unsigned int m_autoDelay;			// Time delay at the start of autonomous mode
	int m_autonEnd;						// Case variable for what the robot will do in autonomous after kicking the zone balls
	bool m_diagnosticMode;		// If 1, robot enters diagnostic mode upon entering teleoperated mode
	
	// last Gamepad button positions
	bool p1_B3old;
	bool p1_B4old;
	bool p1_B5old;
	bool p1_B6old;
	bool p1_B7old;
	bool p1_B8old;
	bool p1_B9old;
	bool p1_B10old;
	bool p2_B4old;
	bool p2_B9old;
	bool p2_DUpold;
	bool p2_DDownold;
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	DriverStationLCD *m_dsLCD;  				 // Driver Station LCD
	
	// Declare variables for the two joysticks being used
	Gamepad *m_gamePad1;			// joystick 1 (arcade stick or right tank stick)
	Gamepad *m_gamePad2;			// joystick 2 (tank left stick)
	
	DashboardDataSender *m_dds;		// Outputs to the dashboard
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;
	
	// Potentiometer values for the 3 basic kicking distances/positions
	static const double kNearshot 	= 3.05;
	static const double kMidshot = 2.7;
	static const double kFarshot = 2.2;	
	double m_farshot;
	
	// Poteniometer values for the 3 basic standing positions
	static const double kTopstand = 2.88;
	static const double kMidstand = 2.2;
	static const double kBottomstand = 1.45;
	static const double kStandmin = 1.3;
	
	double m_topstand;
	double m_midstand;
	double m_bottomstand;
	double m_standmin;
	
	// Coordinates to be used for field locations in autonomous in inches. The center of the field is the origin, the X axis goes along the field's center white line.
	static const double goalX = -324;
	static const double goalY = -144;
	static const double startZoneFarX = 287;
	static const double startRampX = 263;
	static const double farRow1X = 252;
	static const double farRow2X = 216;
	static const double farRow3X = 180;
	static const double midFarBumpX = 126;
	static const double startZoneMidX = 51;
	static const double midRow1X = 36;
	static const double midRow2X = 0;
	static const double midRow3X = -36;
	static const double nearMidBumpX = -90;
	static const double startZoneNearX = -168;
	static const double nearRow1X = -180;
	static const double nearRow2X = -216;
	static const double nearRow3X = -252;
	static const double columnOutY = -117;
	static const double columnMidY = -81;
	static const double columnInY = -45;
	static const double middleLineY = 0;
	static const double tunnelFarX = 180;
	static const double tunnelMidX = 48;
	
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	HotBot2010(void)	{
		printf("Constructor Started\n");
		
		SetPeriod (0.02);	// The program loops at approx. 50 hertz
		
		// PWM
		m_lDrive1 = new Jaguar(1);
		m_rDrive1 = new Jaguar(2);
		m_lDrive2 = new Jaguar(3);
		m_rDrive2 = new Jaguar(4);

		m_kickStand1 = new Jaguar(5);
		m_kickStand2 = new Jaguar(6);
		
		m_lift1 = new Servo(7);
		
		// Relays
		m_kickStand3 = new Relay(1);
		m_shifter = new Relay(2);
		m_roller = new Relay(3);
		
		// Analog Inputs
		m_kickPot = new AnalogChannel(2);
		m_standPot = new AnalogChannel(3);
#ifdef PID_TEST
		testP = 0.2;
		testI = 0.00;
		testD = 0.00;
		
		// Controllers
		m_robotDistance = new HotPID(testP,testI,testD,0.005);
#else
		m_robotDistance = new HotPID(0.075,0.0005,0.001,0.0025);
#endif
		// Controllers
		m_kickerPosition = new HotPID(0.6,0.04,0.0,0.0);
		m_robotTurnBearing = new HotPID(4.50,0.315,0.00,0.00175);
		m_kickerPositionReverse = new HotPID(0.5,0.01,0.0,0.0);		
		m_standerPosition = new HotPID(1.0,0.005,0.0,0.0);
		
		
		m_robotTurnStabilize = new HotPID(0.5,0.15,0.0,0.0);
		
		m_navigate	= new Navigate(LOOPSPERSEC);

		// Create a robot using standard right/left robot drive on PWMS 1, 2, 3, and #4
		m_robotDrive = new RobotDrive(m_lDrive1, m_lDrive2, m_rDrive1, m_rDrive2);

		// Acquire the Driver Station object
		m_ds = DriverStation::GetInstance();
		m_dsLCD = DriverStationLCD::GetInstance();
		
#ifdef DASHBOARD_ON
		m_dds = new DashboardDataSender();
#endif
	
		// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
		m_gamePad1 = new Gamepad(1);
		m_gamePad2 = new Gamepad(2);
		
		// set all Gamepad buttons to not-pressed
		p1_B3old = false;
		p1_B4old = false;
		p1_B5old = false;
		p1_B6old = false;
		p1_B7old = false;
		p1_B8old = false;
		p1_B9old = false;
		p1_B10old = false;
		p2_B4old = false;
		p2_B9old = false;
		p2_DUpold = false;
		p2_DDownold = false;

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;
		
		m_resetFlag = 0;
		m_standResetFlag = 0;
		m_kickingFlag= 0;
		m_joystickFlag = 0;
		m_potFailure = 0;
		m_repeatedKick = 0;
		m_servoed = 0;
		
		m_shotPosition  = kNearshot;
		
		m_standPosition = m_topstand;
		
		m_hangingInstantPot = 0.0;
		
		m_instantPotSet = 0;
		
		m_topstand = 3.2;
		m_midstand = 1.6;
		m_bottomstand = 0.6;
		m_standmin = 0.5;
		
		m_farshot = 1.56;
		
		m_autonomousCase = 1;
		m_autoProgramNumber = 3;
		m_startColumnNo = 3;
		m_startColumn = columnOutY;
		m_autoDelay = 0;
		m_autonEnd = 1;
		m_diagnosticMode = 0;
		
		// Put live camera feed on dashboard
#ifdef CAMERA_ON
		printf("Waiting for camera to boot\n");
		// Wait for the camera to boot
		Wait(5.0);
		printf("Getting camera instance\n");
		AxisCamera &camera = AxisCamera::GetInstance();
		printf("Setting camera parameters\n");
		camera.WriteResolution(AxisCamera::kResolution_320x240);
		camera.WriteCompression(20);
		camera.WriteBrightness(0);
#endif
		
#ifdef WATCHDOG_ON
		GetWatchdog().SetEnabled(true);
#else
		GetWatchdog().SetEnabled(false);
#endif

		printf("BuiltinDefaultCode Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/
	void RobotInit(void) {
		printf("RobotInit() started.\n");
		
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		printf("RobotInit() completed.\n");
			
	}
	
	void DisabledInit(void) {
		printf("DisabledInit() started.\n");
		
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		
		// Reset LCD display to get rid of old lines
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
		m_dsLCD->UpdateLCD();
		
		printf("DisabledInit() completed.\n");
	}

	void AutonomousInit(void) {
		printf("AutonomousInit() started.\n");
		
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		
		m_autonomousCase = 1;					// Reset the state machines for autonomous
	
#ifdef PID_TEST
		// Controllers
		m_robotDistance->SetGains(testP,testI,testD);
#endif
		
		// Set up and enable PIDs
		m_standerPosition->Reset();
		m_standerPosition->SetSPLimits(+5.2, 0.0);	// desired position (volts)
		m_standerPosition->SetPVLimits(+5.2, 0.0);	// actual position (volts)
		m_standerPosition->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_standerPosition->Enable();
						
		m_kickerPosition->Reset();
		m_kickerPosition->SetSPLimits(+5.2, 0.0);	// desired position (volts)
		m_kickerPosition->SetPVLimits(+5.2, 0.0);	// actual position (volts)
		m_kickerPosition->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_kickerPosition->Enable();
		
		m_kickerPositionReverse->Reset();
		m_kickerPositionReverse->SetSPLimits(+5.2, 0.0);	// desired position (volts)
		m_kickerPositionReverse->SetPVLimits(+5.2, 0.0);	// actual position (volts)
		m_kickerPositionReverse->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_kickerPositionReverse->Enable();
		
		m_robotDistance->Reset();
		m_robotDistance->SetSPLimits(+400.0, -400.0);	// desired position (inches)
		m_robotDistance->SetPVLimits(+400.0, -400.0);	// actual position (inches)
		m_robotDistance->SetMVLimits(+0.7, -0.7);	// Manipulated Variable, Jaguar drive
		m_robotDistance->Enable();
								
		m_robotTurnStabilize->Reset();
		m_robotTurnStabilize->SetSPLimits(+1.0, -1.0);	// desired position (volts)
		m_robotTurnStabilize->SetPVLimits(+1.0, -1.0);	// actual position (volts)
		m_robotTurnStabilize->SetMVLimits(+0.3, -0.3);	// Manipulated Variable, Jaguar drive
		m_robotTurnStabilize->Enable();
		
		m_robotTurnBearing->Reset();
		m_robotTurnBearing->SetSPLimits(+4.0, -4.0);	// desired position (volts)
		m_robotTurnBearing->SetPVLimits(+4.0, -4.0);	// actual position (volts)
		m_robotTurnBearing->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_robotTurnBearing->Enable();
	
		m_navigate->Reset();
		
		// Reset LCD display to get rid of old lines
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
		m_dsLCD->UpdateLCD();
		
		// If the robot enters Autonomous mode, it will not be able to go into diagnostic mode - this is a safety
		m_diagnosticMode = 0;
		
		
		printf("AutonomousInit() completed.\n");
	}

	void TeleopInit(void) {
		printf("TeleopInit() started.\n");
		
#ifdef PID_TEST
		// Controllers
		m_robotDistance->SetGains(testP,testI,testD);
#endif
		
		// Set up and enable PIDs
		m_standerPosition->Reset();
		m_standerPosition->SetSPLimits(+6.0, 0.0);	// desired position (volts)
		m_standerPosition->SetPVLimits(+6.0, 0.0);	// actual position (volts)
		m_standerPosition->SetMVLimits(+0.8, -0.8);	// Manipulated Variable, Jaguar drive
		m_standerPosition->Enable();
						
		m_kickerPosition->Reset();
		m_kickerPosition->SetSPLimits(+5.2, 0.0);	// desired position (volts)
		m_kickerPosition->SetPVLimits(+5.2, 0.0);	// actual position (volts)
		m_kickerPosition->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_kickerPosition->Enable();
		
		m_kickerPositionReverse->Reset();
		m_kickerPositionReverse->SetSPLimits(+5.2, 0.0);	// desired position (volts)
		m_kickerPositionReverse->SetPVLimits(+5.2, 0.0);	// actual position (volts)
		m_kickerPositionReverse->SetMVLimits(+1.0, -1.0);	// Manipulated Variable, Jaguar drive
		m_kickerPositionReverse->Enable();
		
		m_navigate->Reset();
		
		// Reset LCD display to get rid of old lines
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "                     ");
		m_dsLCD->UpdateLCD();
		
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		
		m_resetFlag = 0;
		m_standResetFlag = 0;
		m_kickingFlag= 0;
		m_joystickFlag = 0;
		m_potFailure = 0;
		m_repeatedKick = 0;
		m_servoed = 0;
				
		m_shotPosition  = kNearshot;
				
		m_standPosition = m_topstand;
				
		m_hangingInstantPot = 0.0;
				
		m_instantPotSet = 0;
		
		m_instantPotSet = 0;
		
		m_topstand = m_standPot->GetVoltage() + kTopstand;
		m_midstand = m_standPot->GetVoltage() + kMidstand;
		m_bottomstand = m_standPot->GetVoltage() + kBottomstand;
		m_standmin = m_standPot->GetVoltage() + kStandmin;
		
		printf("TeleopInit() completed.\n");
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		
#ifdef WATCHDOG_ON
		// feed the user watchdog at every period when disabled
		GetWatchdog().Feed();
#endif

		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;

#ifdef DASHBOARD_ON		
		// Sends IO data to the dashboard
		m_dds->sendIOPortData(m_shotPosition, m_autoProgramNumber, m_autoDelay);
#endif

#ifdef PID_TEST
		if (m_gamePad1->GetButton04() && m_gamePad1->GetButton06() && (!p1_B6old))
			    {
			    	testP += 0.01;
			    }
			    else if (m_gamePad1->GetButton04() && m_gamePad1->GetButton08() && (!p1_B8old))
			    {
			    	testP -= 0.01;
			    }
			    else if (m_gamePad1->GetButton09() && m_gamePad1->GetButton06() && (!p1_B6old))
			    {
			    	testI += 0.005;
			    }
			    else if (m_gamePad1->GetButton09() && m_gamePad1->GetButton08() && (!p1_B8old))
			    {
			    	testI -= 0.005;
			    }
		if (m_gamePad1->GetButton05() && m_gamePad1->GetButton06() && (!p1_B6old))
				{
					m_autoProgramNumber += 1;
				}
				else if (m_gamePad1->GetButton05() && m_gamePad1->GetButton08() && (!p1_B8old))
				{
					m_autoProgramNumber -= 1;
				}
				if (m_autoProgramNumber < 1) m_autoProgramNumber = 1;
			    m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "P: %4f               ",testP);
			    m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "I: %4f               ",testI);
			    m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Autonomous:%4u          ",m_autoProgramNumber);
#else
		// Hold B5: B6/B8 Increments/Decrements the integer value by 1
		// Hold B7: B6/B8 Increments/Decrements the time by 1
		if (m_gamePad1->GetButton05() && m_gamePad1->GetButton06() && (!p1_B6old))
		{
			m_autoProgramNumber += 1;
		}
		else if (m_gamePad1->GetButton05() && m_gamePad1->GetButton08() && (!p1_B8old))
		{
			m_autoProgramNumber -= 1;
		}
		if (m_autoProgramNumber < 1) m_autoProgramNumber = 1;
		
		else if (m_gamePad1->GetButton07() && m_gamePad1->GetButton06() && (!p1_B6old) && (m_autoDelay < 15))
		{
			m_autoDelay += 1;
		}
		else if (m_gamePad1->GetButton07() && m_gamePad1->GetButton08() && (!p1_B8old) && (m_autoDelay > 0))
		{
			m_autoDelay -= 1;
		}
		
		if ((m_gamePad1->GetButton09() == 1) && (m_diagnosticMode == 0) && (!p1_B9old))
		{
			m_diagnosticMode = 1;
		}
		else if ((m_gamePad1->GetButton09() == 1) && (m_diagnosticMode == 1) && (!p1_B9old))
		{
			m_diagnosticMode = 0;
		}
		
		if ((m_gamePad1->GetButton10() == 1) && (m_navigate->m_useGyro == 0) && (!p1_B10old))
		{
			m_navigate->m_useGyro = 1;
		}
		else if ((m_gamePad1->GetButton10() == 1) && (m_navigate->m_useGyro == 1) && (!p1_B10old))
		{
			m_navigate->m_useGyro = 0;
		}
	    
	    // Select autonomous ball column
	    if (m_gamePad1->GetButton04() && !p1_B4old)
	    {
	       m_startColumnNo++;
	    }
	    if (m_startColumnNo > 3) m_startColumnNo = 1;
	    if (m_startColumnNo < 1) m_startColumnNo = 1;
	    
	    // Select autonomous ball column
	    if (m_gamePad1->GetButton03() && !p1_B3old)
	    {
	    	m_autonEnd++;
	    }
	    if (m_autonEnd > 7) m_autonEnd = 1;
	    if (m_autonEnd < 1) m_autonEnd = 1;
	    
	    if(m_diagnosticMode == 1)
	    {
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Diagnostic: Yes      ");
	    }
	    else
	    {
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Diagnostic: No      ");
	    }
	    
	    m_dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Auto Time Delay:%4u     ",m_autoDelay);
	    
	    
	    switch(m_autoProgramNumber)
	    {
	    case 1:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Autonomous:Near          ");
	    	break;
	    case 2:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Autonomous:Mid          ");
	    	break;
	    case 3:
		   	m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Autonomous:Far          ");
		   	break;
	    default:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Autonomous:None (%2d)      ", m_autoProgramNumber);
		   	break;
	    }
	    
	    switch(m_autonEnd)
	    {
	    case 1:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "AutonEnd: bump+kick2   ");
	    	break;
	    case 2:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "AutonEnd: bump+kick1   ");
	    	break;
	    case 3:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "AutonEnd: bump+stop  ");
	    	break;
	    case 4:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "AutonEnd: stop        ");
	    	break;
	    case 5:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "AutonEnd: bumpx2      ");
	    	break;
	    case 6:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "AutonEnd: B+Kx2+B     ");
	    	break;
	    case 7:
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "AutonEnd: 469          ");
	    	break;
	    }
	    
	    switch(m_startColumnNo)
	    {
	    case 1:
	    	m_startColumn = columnInY;
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Column: Inside         ");
	    	break;
	    case 2:
	    	m_startColumn = columnMidY;
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Column: Middle         ");
	    	break;  	
	    case 3:
	    	m_startColumn = columnOutY;
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Column: Outside        ");
	    	break;
	    default:
	    	break;
	    }
	//    m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Using Gyro:%2d          ",m_navigate->m_useGyro);
	    m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "k,s: %4f,%4f       ", m_kickPot->GetVoltage(),m_standPot->GetVoltage());
	    
#endif
	    m_dsLCD->UpdateLCD();

	    p1_B3old = m_gamePad1->GetButton03();
	    p1_B4old = m_gamePad1->GetButton04();
		p1_B5old = m_gamePad1->GetButton05();
		p1_B6old = m_gamePad1->GetButton06();
	    p1_B7old = m_gamePad1->GetButton07();
	    p1_B8old = m_gamePad1->GetButton08();
	    p1_B9old = m_gamePad1->GetButton09();
	    p1_B10old = m_gamePad1->GetButton10();
	    
	    if((m_disabledPeriodicLoops % 50) == 0)
	    {
	    	printf("Autonomous:%4u          ",m_autoProgramNumber);
	    	printf("Disabled Loops:%5u\n",m_disabledPeriodicLoops);
	    }
	}
	
	void AutonomousPeriodic(void) {
		
#ifdef WATCHDOG_ON
			// feed the user watchdog at every period when in autonomous
			GetWatchdog().Feed();
#endif
			
			m_autoPeriodicLoops++;
			
#ifdef DASHBOARD_ON		
		// Sends IO data to the dashboard
		m_dds->sendIOPortData(m_shotPosition, m_autoProgramNumber, m_autoDelay);
#endif
		
		m_navigate->Update();
		
		m_navigate->ShowCountDriverStation();
				
		m_navigate->ShowDistanceDriverStation();
				
		m_navigate->ShowPositionDriverStation();
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Case:%4d            ",m_autonomousCase);
		m_dsLCD->UpdateLCD();
			
			// Runs a preselected autonomous routine. Select with the gamepad in disabled mode
			switch(m_autoProgramNumber)
			{
				case 1:
					Autonomous6(m_startColumn);
					break;
				case 2:
					Autonomous5(m_startColumn);
					break;
				case 3:
					Autonomous4(m_startColumn);
					break;
				case 4:
					//Autonomous32();
					break;
				case 5: 
					//Autonomous4(m_startColumn);
					break;
				case 6: 
					//GyroTurningTest(m_startColumn, 0.05);
					break;
				case 7: 
					//GyroTurningTest(m_startColumn, 0.15);
					break;
				case 8: 
					//GyroTurningTest(m_startColumn, 1.0);
					break;
				case 9: 
					//DriveToDestinationTest(m_startColumn);
					break;
				default:
					break;
			}

	}

	
	void TeleopPeriodic(void) {
		
#ifdef WATCHDOG_ON
		// feed the user watchdog at every period when in autonomous
		GetWatchdog().Feed();
#endif
		
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;

#ifdef DASHBOARD_ON	
		m_dds->sendIOPortData(m_shotPosition, m_autoProgramNumber, m_autoDelay);
#endif
		
		//m_navigate->Update();
		
		/*switch(m_diagnosticMode)
		{
		case 0:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Diagnostic: No       ");
			break;
		case 1:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Diagnostic: Yes      ");
			break;
		default:
			break;
		}
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "StandPot: %5f       ", m_standPot->GetVoltage());
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "KickPot: %5f       ", m_kickPot->GetVoltage());
		//m_navigate->ShowPositionDriverStation();
		
		 if(m_potFailure == 1)
			{
			   m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Using Pot: No       ");
			}
		 else
			{
			   m_dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "Using Pot: Yes      ");
			}
		m_dsLCD->UpdateLCD();*/
		
		
		// Is the robot in diagnostic mode?
		switch (m_diagnosticMode)
		{
		/****************************************************************************************************************
		* The robot is in normal teleoperated mode (not diagnostic mode)
		***************************************************************************************************************/ 
		case 0:
			
			// If button 9 is pressed, the robot switches into manual control of the shifter in case of a failure
			if ((m_gamePad2->GetButton09() == 1) && (m_potFailure == 0) && (!p2_B9old))
				{
					m_potFailure = 1;
					p2_B9old = 1;
				}
			if ((m_gamePad2->GetButton09() == 1) && (m_potFailure == 1) && (!p2_B9old))
				{
					m_potFailure = 0;
					p2_B9old = 1;
				}
			p2_B9old = m_gamePad2->GetButton09();
			
			/****************************************************************************************************************
			 * Gamepad 1 controls
			 ***************************************************************************************************************/ 
			m_robotDrive->ArcadeDrive(m_gamePad1->GetLeftY(),-(m_gamePad1->GetRightX()),true);
			Roller();
			
			/* if (m_gamePad1->GetButton04() && !p1_B4old)
	    	{
	       		m_farshot = m_farshot + 0.05;
	    	}
	    	if (m_gamePad1->GetButton03() && !p1_B3old)
	    	{
	       		m_farshot = m_farshot - 0.05;
	    	}
	    	p1_B3old = m_gamePad1->GetButton03();
	    	p1_B4old = m_gamePad1->GetButton04();
	    	m_dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "m_farshot = %6f      ", m_farshot);
			m_dsLCD->UpdateLCD(); */
			
			/****************************************************************************************************************
			* Gamepad 2 controls
			***************************************************************************************************************/ 	
			// If Button 8 is pressed, robot enters Stand Mode
			if(m_gamePad2->GetButton08())
			{
				// Button 7 releases the gas servos in Stand Mode - be careful
				if(m_gamePad2->GetButton07() && m_standResetFlag)
				{
					m_servoed = 1;
				}
				if(m_servoed == 1)
				{
					m_lift1->Set(1.0);
				}
				SetStandDistance();
				Stand(m_standPosition);
				StandShifter();
			}
			// If Button 8 is not being held, the kicker is being operated
			else
			{
				// Joystick based control if toggled by Button 4
				SetShotDistance(0);
				if(m_joystickFlag == 1)
				{
					ManualKickerReset();
				}
				// If button 4 is not pressed, semi-automatic controls
				else
				{
					// Holding Buttons 1, 2, and 3 determines what position to set the kicker to
					KickerReset(m_shotPosition);
				}
				// The kicker controls that run in both manual and semi-auto
				KickerRelease(0);
				KickShifter();
			}
			break;
		
		/****************************************************************************************************************
		* Diagnostic Mode
		***************************************************************************************************************/ 
		case 1:
			// Gamepad 1
			m_lDrive1->Set(m_gamePad1->GetLeftX());
			m_rDrive1->Set(m_gamePad1->GetRightX());
			m_lDrive2->Set(m_gamePad1->GetLeftY());
			m_rDrive2->Set(m_gamePad1->GetRightY());
			
			if(m_gamePad1->GetButton01())
			{
				m_lift1->Set(1.0);
			}
			else
			{
				m_lift1->Set(0.0);
			}
			
			if(m_gamePad1->GetButton09())// Turn off diagnostic mode
			{
				m_diagnosticMode = 0;
			}
			
			// Gamepad2
			m_kickStand1->Set(m_gamePad2->GetLeftY());
			m_kickStand2->Set(-m_gamePad2->GetRightY());
			
			if(m_gamePad2->GetButton01())
			{
				m_kickStand3->Set(Relay::kForward);
			}
			else
			{
				m_kickStand3->Set(Relay::kOff);
			}
			
			
			if(m_gamePad2->GetButton03())
			{
				m_shifter->Set(Relay::kReverse);
			}
			else if(m_gamePad2->GetButton02())
			{
				m_shifter->Set(Relay::kForward);
			}
			else
			{
				m_shifter->Set(Relay::kOff);
			}
			
			if(m_gamePad2->GetButton09())// Turn off diagnostic mode
			{
				m_diagnosticMode = 0;
			}
			
			break;
		
		default:
			break;
		} // End diagnostic switch
			
	} // TeleopPeriodic(void)
	
	void DisabledContinuous()
	{
//		printf("Default %s() method... Overload me!\n", __FUNCTION__);
	}

	/**
	 * Continuous code for autonomous mode should go here.
	 *
	 * Users should override this method for code which will be called repeatedly as frequently
	 * as possible while the robot is in autonomous mode.
	 */
	void AutonomousContinuous()
	{
//		printf("Default %s() method... Overload me!\n", __FUNCTION__);
	}

	/**
	 * Continuous code for teleop mode should go here.
	 *
	 * Users should override this method for code which will be called repeatedly as frequently
	 * as possible while the robot is in teleop mode.
	 */
	void TeleopContinuous()
	{
//		printf("Default %s() method... Overload me!\n", __FUNCTION__);
	}
	
	/*************************************************************************************************************************************
	 * Kicker Control Functions
	 ************************************************************************************************************************************/
	
	// 2 PIDs
		void KickerReset(float kickerposition)// m_shotPosition will be inputted for kickerposition
		{
			// Sets up a range of .1 of which the robot will consider the kicker to be reset
			float kickerpositionmax = kickerposition - ALLOWEDPOTENTIOMETERERROR;
			float kickerpositionmin = kickerposition + ALLOWEDPOTENTIOMETERERROR;
			if(((m_kickPot->GetVoltage()) < kickerpositionmin) && ((m_kickPot->GetVoltage()) > kickerpositionmax))
			{
				m_resetFlag = 1;
			}
			else 
			{
				m_resetFlag = 0;
			}
				if(m_kickPot->GetVoltage() - kickerposition > 0)// If the error is positive, run forward
				{
					// Set the kickstand motors to move the kick bar to a position
					m_kickStand1->Set(m_kickerPosition->GetMV(kickerposition,m_kickPot->GetVoltage()));
					m_kickStand2->Set(-m_kickerPosition->GetMV(kickerposition,m_kickPot->GetVoltage()));
					/*if((m_kickerPosition->GetMV(kickerposition,m_kickPot->GetVoltage())) < -0.05)
					{
						m_kickStand3->Set(Relay::kForward);
						m_kickStand3->Set(Relay::kOn);
					}
					else
					{
						m_kickStand3->Set(Relay::kOff);
					}*/
				}
				else// If the kicker has been pulled back too far, run backward
				{
					// Set the kickstand motors to move the kick bar to a position
					m_kickStand1->Set(m_kickerPositionReverse->GetMV(kickerposition,m_kickPot->GetVoltage()));
					m_kickStand2->Set(-m_kickerPositionReverse->GetMV(kickerposition,m_kickPot->GetVoltage()));
					/*if((m_kickerPosition->GetMV(kickerposition,m_kickPot->GetVoltage())) > 0.05)
					{
						m_kickStand3->Set(Relay::kReverse);
						m_kickStand3->Set(Relay::kOn);
					}
					else
					{
						m_kickStand3->Set(Relay::kOff);
					}*/
				}		
		}
/*
	void KickerReset(float kickerposition)// m_shotPosition will be inputted for kickerposition
		{
			// Sets up a range of .1 of which the robot will consider the kicker to be reset
			float kickerpositionmax = kickerposition - ALLOWEDPOTENTIOMETERERROR;
			float kickerpositionmin = kickerposition + ALLOWEDPOTENTIOMETERERROR;
			if(((m_kickPot->GetVoltage()) < kickerpositionmin) && ((m_kickPot->GetVoltage()) > kickerpositionmax))
			{
				m_resetFlag = 1;
			}
			else 
			{
				m_resetFlag = 0;
			}
			if(m_resetFlag == 0)// If the kicker is not reset, KickerReset will move the kicker to the set position. If it is already set, it will not do anything
			{
				if(((m_kickPot->GetVoltage()) - kickerposition) > 0)// If the error is positive, run the normal way
				{
					// Set the kickstand motors to move the kick bar to a position
					m_kickStand1->Set(-0.5);
					m_kickStand2->Set(0.5);
					m_kickStand3->Set(Relay::kForward);
				}
				else// If the kicker has been pulled back too far, run the other way - may not need f-p
				{
					// Set the kickstand motors to move the kick bar to a position
					m_kickStand1->Set(0.1);
					m_kickStand2->Set(-0.1);
					m_kickStand3->Set(Relay::kReverse);
				}
				
			}
			else
			{
				m_kickStand1->Set(-0.1);
				m_kickStand2->Set(0.1);
				m_kickStand3->Set(Relay::kOff);
			}

		}*/
	
	void ManualKickerReset (void)
	{
		if(m_standPot->GetVoltage() < STANDBOTTOM)
		{
		if((m_gamePad2->GetRightY() > 0.1) && ((m_kickPot->GetVoltage() < POTMAXVALUE) || (m_potFailure == 1)))	// If the joystick is moved and the kicker is within the safe range
		{
			m_kickStand1->Set(m_gamePad2->GetRightY());
			m_kickStand2->Set(-m_gamePad2->GetRightY());
			m_kickStand3->Set(Relay::kForward);
			m_resetFlag = 0;
		}
		else if((m_gamePad2->GetRightY() < -0.1) && ((m_kickPot->GetVoltage() > POTMINVALUE) || (m_potFailure == 1))) // If the joystick is moved and the kicker is within the safe range
		{
			m_kickStand1->Set(m_gamePad2->GetRightY());
			m_kickStand2->Set(-m_gamePad2->GetRightY());
			m_kickStand3->Set(Relay::kReverse);
			m_resetFlag = 0;
		}
		else
		{
			m_kickStand1->Set(-0.1);
			m_kickStand2->Set(0.1);
			m_kickStand3->Set(Relay::kOff);
			m_resetFlag = 1;
		}
		}
	}
	
	// This shifter control runs until the pot reads greater than the bottom value - an indication that the shifter has reengaged	
	void KickShifter(void)
	{
		if(m_servoed == 1)
		{
			if(m_instantPotSet == 0)
			{
				m_instantPotSet = 1;
				m_hangingInstantPot = m_standPot->GetVoltage();
			}
			double hangtolerancemax = m_hangingInstantPot + 0.2;
			double hangtolerancemin = m_hangingInstantPot - 0.2;
			if(m_standPot->GetVoltage() > hangtolerancemax || m_standPot->GetVoltage() < hangtolerancemin)
			{
				m_shifter->Set(Relay::kOff);
			}
			else
			{
				m_shifter->Set(Relay::kReverse);
			}
			m_kickStand1->Set(0.0);
			m_kickStand2->Set(0.0);
		}
		else
		{
			if((m_kickingFlag == 1 && m_kickPot->GetVoltage() < KICKERBOTTOM) || ((m_potFailure == 1) && (m_gamePad2->GetButton06())))
			{
				m_shifter->Set(Relay::kForward);
			}
			else if(((m_resetFlag == 0) && (m_kickPot->GetVoltage() > KICKERBOTTOM)) || ((m_potFailure == 1) && (m_gamePad2->GetButton05())))
			{
				m_shifter->Set(Relay::kReverse);
			}
			else
			{
				m_shifter->Set(Relay::kOff);
			}
		}
	}
	void StandShifter(void)
	{
		if(m_gamePad2->GetLeftX() > 0.5 || m_gamePad2->GetLeftX() < -0.5)
		{
			if(m_gamePad2->GetLeftX() > 0.5) m_shifter->Set(Relay::kReverse);
			if(m_gamePad2->GetLeftX() < -0.5) m_shifter->Set(Relay::kForward);
		}
		else
		{
			if((m_standPot->GetVoltage() < m_standmin) && (m_standResetFlag == 0))
			{
			m_shifter->Set(Relay::kForward);	
			}
			else
			{
				m_shifter->Set(Relay::kOff);
			}
		}
	}
	void SetShotDistance(int selectkickposition)// selectkickposition = 0 for teleoperated and 1,2,or 3 for autonomous to select a shot distance
	{
		/*************************************************************************************************************************
		 * Button 4 toggles joystick control
		 * If Buttons 2 or 3 are held, kicker will move to those setpoints
		 * If the robot is in setpoint mode but Buttons 2 or 3 are not held, the robot returns to the lowest setpoint 
		 * 		so motors are not abused
		 * Note that if the robot is in setpoint mode, Button 1 effectively does nothing as the default position is 1
		 * To fire the kicker in setpoint mode, you must hold Button 1, 2, or 3 AND the shoot button (6), and it will
		 * 		not shoot until the kicker reaches the correct position
		 * **********************************************************************************************************************/
		if(selectkickposition == 0)
		{
			if((m_gamePad2->GetButton04() && m_joystickFlag == 0 && !p2_B4old) || (m_potFailure == 1))
			{
				m_joystickFlag = 1;
			}
			else if((m_gamePad2->GetButton04() && m_joystickFlag == 1 && !p2_B4old) || (m_potFailure == 1))
			{
				m_joystickFlag = 0;
			}
			else if(m_gamePad2->GetButton03())
			{
				m_shotPosition = kFarshot;
				// m_shotPosition = m_farshot;
			}
			else if(m_gamePad2->GetButton02())
			{
				m_shotPosition = kMidshot;
			}
			else
			{
				m_shotPosition = kNearshot;
			}
			p2_B4old = m_gamePad2->GetButton04();
		}
		else if (selectkickposition == 1)
		{
			m_shotPosition = kNearshot;
		}
		else if (selectkickposition == 2)
		{
			m_shotPosition = kMidshot;
		}
		else if (selectkickposition == 3)
		{
			m_shotPosition = kFarshot;
		}
	}
	void KickerRelease(bool autokick)// If autokick is 1 (in autonomous, for instance) the robot can kick without user input
	{
			if(!(m_gamePad2->GetButton06()))
			{
				m_repeatedKick = 0;
			}
			if(m_kickingFlag == 1)// If the robot has started to kick
			{
				// The robot knows it has stopped kicking if the kicker reads a bottom pot value
				if((m_kickPot->GetVoltage() > KICKERBOTTOM) || (m_potFailure == 1))
				{
					m_kickingFlag = 0;
				}
			}
			/******************************************************************************************************************
			 *  The robot will start semiautonomous kicking if:
			 *	1. An argument is specified (for autonomous mode)
			 *  2. Anytime in joystick mode the operator presses button 6 and the pot has not failed
			 *  3. In setpoint mode when the operator presses button 6 and the kicker has reached the desired location
			 *     (If the pot fails, setpoint mode cannot be entered)(Button 1, 2, or 3 must be held to enable shooting)
			*******************************************************************************************************************/
			else if( ((m_repeatedKick == 0 && m_resetFlag == 1 && (m_gamePad2->GetButton01() || m_gamePad2->GetButton02() || m_gamePad2->GetButton03() || m_joystickFlag == 1) && m_potFailure == 0 && m_gamePad2->GetButton06())) || autokick == 1)
			{
				m_kickingFlag = 1;
				m_repeatedKick = 1;
				
			}
	}
	void Roller(void)
	{
		if(m_gamePad1->GetButton06())
		{
			m_roller->Set(Relay::kForward);
		}
		else
		{
			m_roller->Set(Relay::kReverse);
		}
	}
	void Stand(double positionselect)
	{
		float standpositionmax = positionselect + ALLOWEDPOTENTIOMETERERRORSTAND;
		float standpositionmin = positionselect - ALLOWEDPOTENTIOMETERERRORSTAND;
		if(((m_standPot->GetVoltage()) > standpositionmin) && ((m_standPot->GetVoltage()) < standpositionmax))
		{
			m_standResetFlag = 1;
		}
		else 
		{
			m_standResetFlag = 0;
		}
	/*	if(m_gamePad2->GetLeftY() > 0.1 && m_standPot->GetVoltage() > kBottomstand)
		{
			m_kickStand1->Set(-m_gamePad2->GetLeftY());
			m_kickStand2->Set(m_gamePad2->GetLeftY());
			//m_kickStand3->Set(Relay::kForward);
			m_standResetFlag = 0;
		}
		else if(m_gamePad2->GetLeftY() < -0.1)
		{
			m_kickStand1->Set(-m_gamePad2->GetLeftY());
			m_kickStand2->Set(m_gamePad2->GetLeftY());
			//m_kickStand3->Set(Relay::kReverse);
			m_standResetFlag = 0;
		}			*/	
		//else
		//{
			m_kickStand1->Set(m_standerPosition->GetMV(positionselect,m_standPot->GetVoltage()));
			m_kickStand2->Set(-m_standerPosition->GetMV(positionselect,m_standPot->GetVoltage()));
			//m_kickStand3->Set(Relay::kForward);
		//}
	}
	void SetStandDistance(void)
	{
		/*************************************************************************************************************************
		 * If Buttons 2 or 3 are held, stander will move to those setpoints
		 * If the robot is in setpoint mode but Buttons 2 or 3 are not held, the robot returns to the lowest setpoint 
		 * 		so motors are not abused
		 * Note that if the robot is in setpoint mode, Button 1 effectively does nothing as the default position is 1
		 * To stand in setpoint mode, you must hold Button 1, 2, or 3 AND the stand button (7), and it will
		 * 		not shoot until the stander reaches the correct position
		 * **********************************************************************************************************************/

		//if(m_gamePad2->GetButton06())
	//	{
		//	m_standPosition = m_midstand;
	//	}
	//	else
	//	{
			m_standPosition = m_topstand;
	//	}
		if ((m_gamePad2->GetDpadY() > 0.5) && !p2_DUpold)
		{
			m_topstand = m_topstand - 0.1;
			p2_DUpold = true;
		}
		else if (m_gamePad2->GetDpadY() < 0.5 && m_gamePad2->GetDpadY() > -0.5)
		{
			p2_DUpold = false;
		}
		if ((m_gamePad2->GetDpadY() < -0.5) && !p2_DDownold)
		{
			m_topstand = m_topstand + 0.1;
			p2_DDownold = true;
		}
		else if (m_gamePad2->GetDpadY() < 0.5 && m_gamePad2->GetDpadY() > -0.5)
		{
			p2_DDownold = false;
		}
	}
/******************************************************************************************************/
	// Autonomous Modules
	bool DriveToDestination(void)
	{
		double drivespeed;
		double turnspeed;
		
		double deltaX = m_navigate->m_position_x - m_navigate->m_destination_x;
		double deltaY = m_navigate->m_position_y - m_navigate->m_destination_y;
		
		double drivedistance = sqrt((deltaX*deltaX) + (deltaY*deltaY));
		if(deltaX < 0) drivedistance = -drivedistance;
		m_navigate->m_bearing = atan2(deltaY,deltaX);
		
		drivespeed = m_robotDistance->GetMV(drivedistance,0.0);
		turnspeed = m_robotTurnStabilize->GetMV(m_navigate->m_bearing,m_navigate->m_heading);
		m_robotDrive->TankDrive(drivespeed - turnspeed,drivespeed + turnspeed);
		if(m_robotDistance->OnTarget()) return true;
		else return false;
	}
	bool kDriveToDestination(void)
	{
		double drivespeed;
		double turnspeed;
			
		double deltaX = m_navigate->m_position_x - m_navigate->m_destination_x;
		double deltaY = m_navigate->m_position_y - m_navigate->m_destination_y;
			
		double drivedistance = sqrt((deltaX*deltaX) + (deltaY*deltaY));
		if(deltaX < 0) drivedistance = -drivedistance;
		m_navigate->m_bearing = atan2(deltaY,deltaX);
			
		drivespeed = 0.6;
		turnspeed = m_robotTurnStabilize->GetMV(m_navigate->m_bearing,m_navigate->m_heading);
		m_robotDrive->TankDrive(drivespeed - turnspeed,drivespeed + turnspeed);
		if (drivedistance < 0.0) return true;
		else return false;
	}
	bool TurnToBearing(void)
	{
		double turnspeed;
		turnspeed = m_robotTurnBearing->GetMV(m_navigate->m_bearing,m_navigate->m_heading);
		m_robotDrive->TankDrive(-turnspeed,turnspeed);
		if(m_robotTurnBearing->OnTarget()) return true;
		else return false;
	}
	void VariableSetShotDistance(double position)
	{
		position = KICKERBOTTOM - position;
		position = position / cos(m_navigate->m_heading);
		position = KICKERBOTTOM - position;
		m_shotPosition = position;
	}
	bool AutonomousKicker(bool kick, double position)
	{
		Roller();
		VariableSetShotDistance(position);
		KickShifter();
		KickerReset(m_shotPosition);
		KickerRelease(kick);
		if(m_kickingFlag == 0) return true;
		else return false;
	}
	
/******************************************************************************************************/

	  	//Far Zone ---- WITHOUT TURNING BACK AFTER SHOOTING
	 void Autonomous4(double column)
	{
		double drivespeed = 0.8;
		//double turnspeed = 0.7;
		double setupangle = 0.0;
		
		double deltaX = m_navigate->m_position_x - goalX;
		double deltaY = m_navigate->m_position_y - goalY;
		
		switch(m_autonomousCase)
			{
			case 1:
				if(m_autonEnd == 5 || m_autonEnd == 6 || m_autonEnd == 7) m_autonomousCase = 40;
				if(column < -90) m_navigate->m_position_x = startRampX;
				else m_navigate->m_position_x = startZoneFarX;
				m_navigate->m_position_y = column;
				m_navigate->m_destination_x = farRow1X;
				m_navigate->m_destination_y = column;
				if(column > -90) m_robotTurnBearing->SetGains(2.0,0.25,0.00);
				if(column > -50) m_robotTurnBearing->SetGains(1.8,0.2,0.00);
				AutonomousKicker(0,2.4);
				m_robotDrive->TankDrive(0.0,0.0);
				if(m_autoDelay * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 2;
				break;
			case 2:
				AutonomousKicker(0,2.4);
				if(DriveToDestination()) m_autonomousCase = 3;
				break;
			case 3:
				AutonomousKicker(0,2.4);
				m_navigate->m_bearing = atan2(deltaY,deltaX);
				if(TurnToBearing()) m_autonomousCase = 4;
				break;
			case 4:
				AutonomousKicker(1,2.4);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 5;
				break;
			case 5:
				m_robotDrive->TankDrive(0.0,0.0);
				if(AutonomousKicker(0,2.4)) m_autonomousCase = 7;
				break;
			case 7:
				m_navigate->m_destination_x = farRow2X;
				AutonomousKicker(0,2.5);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 8;
				break;
			case 8:
				AutonomousKicker(0,2.5);
				if(DriveToDestination()) m_autonomousCase = 9;
				break;
			case 9:
				AutonomousKicker(0,2.5);
				m_navigate->m_bearing = atan2(deltaY,deltaX);
				if(TurnToBearing()) m_autonomousCase = 10;
				break;
			case 10:
				AutonomousKicker(1,2.5);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 11;
				break;
			case 11:
				m_robotDrive->TankDrive(0.0,0.0);
				if(AutonomousKicker(0,2.5)) m_autonomousCase = 13;
				break;
			case 13:
				m_navigate->m_destination_x = farRow3X;
				AutonomousKicker(0,2.55);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 14;
				break;
			case 14:
				AutonomousKicker(0,2.55);
				if(DriveToDestination()) m_autonomousCase = 15;
				break;
			case 15:
				AutonomousKicker(0,2.55);
				m_navigate->m_bearing = atan2(deltaY,deltaX);
				if(TurnToBearing()) m_autonomousCase = 16;
				break;
			case 16:
				AutonomousKicker(1,2.55);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 17;
				break;
			case 17:
				m_robotDrive->TankDrive(0.0,0.0);
				if(AutonomousKicker(0,2.55) && (m_autonEnd == 1 || m_autonEnd == 2 || m_autonEnd == 3)) m_autonomousCase = 19;
				if(AutonomousKicker(0,2.55) && m_autonEnd == 4) m_autonomousCase = 33;
				break;
				// Go over bump??!
			case 19:
				m_navigate->m_destination_x = midFarBumpX;
				m_navigate->m_position_y = column + 2;
				AutonomousKicker(0,2.62);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 20;
				break;
			case 20:
				AutonomousKicker(0,2.62);
				if(DriveToDestination()) m_autonomousCase = 21;
				break;
			case 21:
				m_navigate->Reset();
				AutonomousKicker(0,2.62);
				m_robotDrive->TankDrive(drivespeed,drivespeed);
				m_autonomousCase = 22;
				break;
			case 22:
				AutonomousKicker(0,2.62);
				m_robotDrive->TankDrive(drivespeed,drivespeed);
				if((m_navigate->m_leftDrive->GetDistance() > 83) && (m_autonEnd == 1 || m_autonEnd == 2 || m_autonEnd == 6)) m_autonomousCase = 23;
				if((m_navigate->m_leftDrive->GetDistance() > 83) && m_autonEnd == 3) m_autonomousCase = 33;
				if((m_navigate->m_leftDrive->GetDistance() > 83) && m_autonEnd == 5) m_autonomousCase = 35;
				if((m_navigate->m_leftDrive->GetDistance() > 83) && m_autonEnd == 7) m_autonomousCase = 58;
				break;
			case 23:
				m_navigate->Reset();
				m_navigate->m_position_x = startZoneMidX + 3;
				m_navigate->m_destination_x = midRow1X;
				m_navigate->m_position_y = column + 0.66;
				m_navigate->m_destination_y = column;
				m_autoPeriodicLoops = 0;
				AutonomousKicker(0,2.62);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 34;
				break;
			case 34:
				AutonomousKicker(0,2.62);
				m_robotDrive->TankDrive(0.0,0.0);
				if(0.2 * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 24;
				break;
			case 24:
				AutonomousKicker(0,2.62);
				if(DriveToDestination()) m_autonomousCase = 26;
				break;
		//	case 25:
		//		AutonomousKicker(0,2.2);
		//		m_navigate->m_bearing = atan2(deltaY,deltaX);
		//		if(TurnToBearing()) m_autonomousCase = 26;
		//		break;
			case 26:
				AutonomousKicker(1,2.62);
				m_robotDrive->TankDrive(0.3,0.3);
				m_autonomousCase = 27;
				break;
			case 27:
				m_robotDrive->TankDrive(0.3,0.3);
				if(AutonomousKicker(0,2.62) && (m_autonEnd == 1 || m_autonEnd == 6)) m_autonomousCase = 29;
				if(AutonomousKicker(0,2.62) && m_autonEnd == 2) m_autonomousCase = 33;
				break;
			case 29:
				AutonomousKicker(0,2.75);
				m_robotDrive->TankDrive(0.6,0.6);
				m_autonomousCase = 30;
				break;
			case 30:
				AutonomousKicker(0,2.75);
				m_robotDrive->TankDrive(0.65,0.65);
				if(m_navigate->m_position_x < -39) m_autonomousCase = 31;
				break;
			case 31:
				AutonomousKicker(1,2.75);
				m_robotDrive->TankDrive(0.65,0.65);
				m_autonomousCase = 32;
				break;
			case 32:
				m_robotDrive->TankDrive(0.5,0.5);
				if(AutonomousKicker(0,2.81) && m_autonEnd == 1) m_autonomousCase = 33;
				if(AutonomousKicker(0,2.81) && m_autonEnd == 6) m_autonomousCase = 35;
				break;
			case 33:
				m_robotDrive->TankDrive(0.0,0.0);
				AutonomousKicker(0,2.81);
				break;
				
			// Extra Side-Path for Second Bump
			case 35:
				m_navigate->Reset();
				m_navigate->m_position_x = startZoneMidX;
				m_navigate->m_destination_x = nearMidBumpX;
				m_navigate->m_position_y = column;
				m_navigate->m_destination_y = column;
				m_autoPeriodicLoops = 0;
				AutonomousKicker(0,3.0);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 36;
				break;
			case 36:
				AutonomousKicker(0,3.0);
				m_robotDrive->TankDrive(0.0,0.0);
				if(0.0 * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 38;
				break;
			case 38:
				m_navigate->Reset();
				AutonomousKicker(0,3.0);
				m_robotDrive->TankDrive(1.0,1.0);
				m_autonomousCase = 39;
				break;
			case 39:
				AutonomousKicker(0,3.0);
				m_robotDrive->TankDrive(1.0,1.0);
				if(m_navigate->m_leftDrive->GetDistance() > 300) m_autonomousCase = 33;
				break;
				
				// Fast Autonomous
				
			case 40:
				if(column < -90) m_navigate->m_position_x = startRampX;
				else m_navigate->m_position_x = startZoneFarX;
				m_navigate->m_position_y = column;
				m_navigate->m_destination_x = farRow1X + 5;
				deltaX = farRow1X - goalX;
				deltaY = column - goalY;
				setupangle = atan2(deltaY,deltaX);
				m_navigate->m_destination_y = column + 5*tan(setupangle);
				AutonomousKicker(0,2.3);
				m_robotDrive->TankDrive(0.0,0.0);
				if((m_autoDelay + 0.5) * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 41;
				break;
			case 41:
				AutonomousKicker(0,2.3);
				if(kDriveToDestination()) m_autonomousCase = 42;
				break;
			case 42:
				 AutonomousKicker(0,2.3);
				m_navigate->m_destination_x = farRow1X;
				m_navigate->m_destination_y = column;
				m_autonomousCase = 43;
				break;
			case 43:
				AutonomousKicker(0,2.3);
				if(kDriveToDestination()) m_autonomousCase = 44;
				break;
			case 44:
				AutonomousKicker(1,2.3);
				m_robotDrive->TankDrive(0.5,0.5);
				m_autonomousCase = 45;
				break;
			case 45:
				m_robotDrive->TankDrive(0.5,0.5);
				if (AutonomousKicker(0,2.3)) m_autonomousCase = 46;
				break;
			case 46:
				m_navigate->m_destination_x = farRow2X + 5;
				deltaX = farRow2X - goalX;
				deltaY = column - goalY;
				setupangle = atan2(deltaY,deltaX);
				m_navigate->m_destination_y = column + 5*tan(setupangle);
				AutonomousKicker(0,2.4);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 47;
				break;
			case 47:
				AutonomousKicker(0,2.4);
				if(kDriveToDestination()) m_autonomousCase = 48;
				break;
			case 48:
				AutonomousKicker(0,2.4);
				m_navigate->m_destination_x = farRow2X;
				m_navigate->m_destination_y = column;
				m_autonomousCase = 49;
				break;
			case 49:
				AutonomousKicker(0,2.4);
				if(kDriveToDestination()) m_autonomousCase = 50;
				break;
			case 50:
				AutonomousKicker(1,2.4);
				m_robotDrive->TankDrive(0.5,0.5);
				m_autonomousCase = 51;
				break;
			case 51:
				m_robotDrive->TankDrive(0.5,0.5);
				if (AutonomousKicker(0,2.4)) m_autonomousCase = 52;
				break;
			case 52:
				m_navigate->m_destination_x = farRow3X + 5;
				deltaX = farRow3X - goalX;
				deltaY = column - goalY;
				setupangle = atan2(deltaY,deltaX);
				m_navigate->m_destination_y = column + 5*tan(setupangle);
				AutonomousKicker(0,2.5);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 53;
				break;
			case 53:
				AutonomousKicker(0,2.5);
				if(kDriveToDestination()) m_autonomousCase = 54;
				break;
			case 54:
				AutonomousKicker(0,2.5);
				m_navigate->m_destination_x = farRow3X;
				m_navigate->m_destination_y = column;
				m_autonomousCase = 55;
				break;
			case 55:
				AutonomousKicker(0,2.5);
				if(kDriveToDestination()) m_autonomousCase = 56;
				break;
			case 56:
				AutonomousKicker(1,2.5);
				m_robotDrive->TankDrive(0.5,0.5);
				m_autonomousCase = 57;
				break;
			case 57:
				m_robotDrive->TankDrive(0.5,0.5);
				if(AutonomousKicker(0,2.5)) m_autonomousCase = 19;
				break;
				
			// 469
			case 58:
				m_navigate->Reset();
				m_navigate->m_position_x = startZoneMidX;
				m_navigate->m_destination_x = startZoneMidX;
				m_navigate->m_position_y = middleLineY;
				m_navigate->m_destination_y = column;
				AutonomousKicker(0,3.0);
				m_robotDrive->TankDrive(0.0,0.0);
				m_autonomousCase = 59;
				break;
			case 59:
				AutonomousKicker(0,3.0);
				if(DriveToDestination()) m_autonomousCase = 33;
				break;
				 					
			default:
				AutonomousKicker(0,3.0);
				m_robotDrive->TankDrive(0.0,0.0);
				break;
			}	
		
	}
	 	//Mid Zone
	 	 void Autonomous5(double column)
	 	{
	 		double drivespeed = 0.8;
	 		//double turnspeed = 0.7;
	 		
	 		double deltaX = m_navigate->m_position_x - goalX;
	 		double deltaY = m_navigate->m_position_y - goalY;
	 		
	 		switch(m_autonomousCase)
	 			{
	 			case 1:
	 				if(m_autonEnd == 6) m_autonEnd = 1;
	 				m_navigate->m_position_x = startZoneMidX;
	 				m_navigate->m_position_y = column;
	 				m_navigate->m_destination_x = midRow1X;
	 				m_navigate->m_destination_y = column;
	 				if(column > -90) m_robotTurnBearing->SetGains(1.6,0.15,0.00);
	 				if(column > -50) m_robotTurnBearing->SetGains(1.4,0.1,0.00);
	 				AutonomousKicker(0,2.62);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				if(m_autoDelay * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 2;
	 				break;
	 			case 2:
	 				AutonomousKicker(0,2.62);
	 				if(DriveToDestination()) m_autonomousCase = 3;
	 				break;
	 			case 3:
	 				AutonomousKicker(0,2.62);
	 				m_navigate->m_bearing = atan2(deltaY,deltaX);
	 				if(TurnToBearing()) m_autonomousCase = 4;
	 				break;
	 			case 4:
	 				AutonomousKicker(1,2.62);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				m_autonomousCase = 5;
	 				break;
	 			case 5:
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				if(AutonomousKicker(0,2.62) && (column < -90)) m_autonomousCase = 7;
	 				if(AutonomousKicker(0,2.62)) m_autonomousCase = 6;
	 				break;
	 			case 6:
	 				AutonomousKicker(0,2.75);
	 				m_navigate->m_bearing = 0.0;
	 				if(TurnToBearing()) m_autonomousCase = 7;
	 				break;
	 			case 7:
	 				m_navigate->m_destination_x = midRow2X;
	 				AutonomousKicker(0,2.75);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				m_autonomousCase = 8;
	 				break;
	 			case 8:
	 				AutonomousKicker(0,2.75);
	 				if(DriveToDestination()) m_autonomousCase = 9;
	 				break;
	 			case 9:
	 				AutonomousKicker(0,2.75);
	 				m_navigate->m_bearing = atan2(deltaY,deltaX);
	 				if(TurnToBearing()) m_autonomousCase = 10;
	 				break;
	 			case 10:
	 				AutonomousKicker(1,2.75);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				m_autonomousCase = 11;
	 				break;
	 			case 11:
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				if(AutonomousKicker(0,2.75)) m_autonomousCase = 12;
	 				break;
	 			case 12:
	 				AutonomousKicker(0,3.0);
	 				m_navigate->m_bearing = 0.0;
	 				deltaX = m_navigate->m_position_x - startZoneMidX;
	 				deltaY = m_navigate->m_position_y - middleLineY;
	 				if(m_autonEnd == 7) m_navigate->m_bearing = atan2(deltaY, deltaX);
	 				if(TurnToBearing() && (m_autonEnd == 1 || m_autonEnd == 2 || m_autonEnd == 3 || m_autonEnd == 5)) m_autonomousCase = 13;
	 				if(TurnToBearing() && m_autonEnd == 4) m_autonomousCase = 23;
	 				if(TurnToBearing() && m_autonEnd == 7) m_autonomousCase = 24;
	 				break;
	 				// Go over bump??!
	 			case 13:
	 				m_navigate->m_destination_x = nearMidBumpX;
	 				m_navigate->m_position_y = column - 2;
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				m_autonomousCase = 14;
	 				break;
	 			case 14:
	 				AutonomousKicker(0,3.0);
	 				if(DriveToDestination()) m_autonomousCase = 15;
	 				break;
	 			case 15:
	 				m_navigate->Reset();
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(drivespeed,drivespeed);
	 				m_autonomousCase = 16;
	 				break;
	 			case 16:
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(drivespeed,drivespeed);
	 				if((m_navigate->m_leftDrive->GetDistance() > 85) && (m_autonEnd == 1 || m_autonEnd == 2)) m_autonomousCase = 17;
	 				if((m_navigate->m_leftDrive->GetDistance() > 85) && (m_autonEnd == 3 || m_autonEnd == 5)) m_autonomousCase = 23;
	 				break;
	 			case 17:
	 				m_navigate->Reset();
	 				m_navigate->m_position_x = startZoneNearX;
	 				m_navigate->m_destination_x = nearRow1X;
	 				m_navigate->m_position_y = column + 1;
	 				m_navigate->m_destination_y = column;
	 				m_autoPeriodicLoops = 0;
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(drivespeed,drivespeed);
	 				m_autonomousCase = 18;
	 				break;
	 			case 18:
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				if(0.2 * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 19;
	 				break;
	 			case 19:
	 				AutonomousKicker(0,3.0);
	 				if(DriveToDestination()) m_autonomousCase = 21;
	 				break;
	 			case 21:
	 				AutonomousKicker(1,3.0);
	 				m_robotDrive->TankDrive(0.3,0.3);
	 				m_autonomousCase = 22;
	 				break;
	 			case 22:
	 				m_robotDrive->TankDrive(0.3,0.3);
	 				if(AutonomousKicker(0,3.0)) m_autonomousCase = 23;
	 				break;
	 			case 23:
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				break;
	 				
	 				// 469
	 			case 24:
	 				m_navigate->m_destination_x = startZoneMidX;
	 				m_navigate->m_destination_y = middleLineY;
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				m_autonomousCase = 25;
	 				break;
	 			case 25:
	 				AutonomousKicker(0,3.0);
	 				if(DriveToDestination()) m_autonomousCase = 23;
	 				break;
	 			default:
	 				AutonomousKicker(0,3.0);
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				break;
	 			}	
	 		
	 	}
	 	//Near Zone
	 		 	 void Autonomous6(double column)
	 		 	{
	 		 		//double turnspeed = 0.7;
	 		 		
	 		 		double deltaX = m_navigate->m_position_x - goalX;
	 		 		double deltaY = m_navigate->m_position_y - goalY;
	 		 		
	 		 		switch(m_autonomousCase)
	 		 			{
	 		 			case 1:
	 		 				m_navigate->m_position_x = startZoneNearX;
	 		 				m_navigate->m_position_y = column;
	 		 				m_navigate->m_destination_x = nearRow1X;
	 		 				m_navigate->m_destination_y = column;
	 		 				m_robotTurnBearing->SetGains(2.0,0.2,0.00);
	 		 				if(column > -90) m_robotTurnBearing->SetGains(1.4,0.1,0.00);
	 		 				if(column > -50) m_robotTurnBearing->SetGains(1.0,0.5,0.00);
	 		 				AutonomousKicker(0,3.0);
	 		 				m_robotDrive->TankDrive(0.0,0.0);
	 		 				if(m_autoDelay * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 2;
	 		 				break;
	 		 			case 2:
	 		 				AutonomousKicker(0,3.0);
	 		 				if(DriveToDestination()) m_autonomousCase = 3;
	 		 				break;
	 		 			case 3:
	 		 				AutonomousKicker(0,3.0);
	 		 				m_navigate->m_bearing = atan2(deltaY,deltaX);
	 		 				if(TurnToBearing()) m_autonomousCase = 4;
	 		 				break;
	 		 			case 4:
	 		 				AutonomousKicker(1,3.0);
	 		 				m_robotDrive->TankDrive(0.0,0.0);
	 		 				m_autonomousCase = 5;
	 		 				break;
	 		 			case 5:
	 		 				m_robotDrive->TankDrive(0.0,0.0);
	 		 				if(AutonomousKicker(0,3.0)) m_autonomousCase = 6;
	 		 				break;
	 		 			case 6:
	 		 				AutonomousKicker(0,3.0);
	 		 				m_robotDrive->TankDrive(0.0,0.0);
	 		 				break;
	 		 			default:
	 		 				AutonomousKicker(0,3.0);
	 		 				m_robotDrive->TankDrive(0.0,0.0);
	 		 				break;
	 		 			}	
	 		 		
	 		 	}
	 void GyroTurningTest(double column,double bearing)
		{
			
			switch(m_autonomousCase)
				{
				case 1:
					if(column < -90) m_navigate->m_position_x = startRampX;
					else m_navigate->m_position_x = startZoneFarX;
					m_navigate->m_position_y = column;
					m_navigate->m_destination_x = farRow1X;
					m_navigate->m_destination_y = column;
					Roller();
					SetShotDistance(1);
					KickShifter();
					KickerReset(m_shotPosition);
					KickerRelease(0);
					m_robotDrive->TankDrive(0.0,0.0);
					m_autonomousCase = 2;
					break;
				case 2:
					Roller();
					SetShotDistance(1);
					KickShifter();
					KickerReset(m_shotPosition);
					KickerRelease(0);
					//m_navigate->m_bearing = atan2(deltaY,deltaX);
					m_navigate->m_bearing = bearing;
					if(TurnToBearing()) m_autonomousCase = 3;
					break;
				case 3:
					Roller();
					SetShotDistance(1);
					KickShifter();
					KickerReset(m_shotPosition);
					KickerRelease(0);
					m_robotDrive->TankDrive(0.0,0.0);
					break;
				default:
					break;
					
				}	
		}
	 void DriveToDestinationTest(double column)
	 {
				switch(m_autonomousCase)
					{
					case 1:
						if(column < -90) m_navigate->m_position_x = startRampX;
						else m_navigate->m_position_x = startZoneFarX;
						m_navigate->m_position_y = columnMidY;
						m_navigate->m_destination_x = farRow1X;
						m_navigate->m_destination_y = columnMidY;
						Roller();
						SetShotDistance(1);
						KickShifter();
						KickerReset(m_shotPosition);
						KickerRelease(0);
						m_robotDrive->TankDrive(0.0,0.0);
						m_autonomousCase = 2;
						break;
					case 2:
						Roller();
						SetShotDistance(1);
						KickShifter();
						KickerReset(m_shotPosition);
						KickerRelease(0);
						DriveToDestination();
						break;
					default:
						break;
						
					}	
			}
	 // park in tower
	 	 void Autonomous469(double column)
	 	{
	 		
	 		switch(m_autonomousCase)
	 			{
	 			case 1:
	 				m_navigate->m_position_x = startZoneMidX;
	 				m_navigate->m_position_y = column;
	 				m_navigate->m_destination_x = startZoneMidX;
	 				m_navigate->m_destination_y = middleLineY;
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				if(m_autoDelay * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 2;
	 				break;
	 			case 2:
	 				if(DriveToDestination()) m_autonomousCase = 3;
	 				break;
	 			case 3:
	 				m_robotDrive->TankDrive(0.0,0.0);
	 				break;
	 			default:
	 				break;
	 			}
	 	}
	 	 
	 	void FastAutonomous4(double column)
	 		{
	 			double drivespeed = 0.8;
	 			//double turnspeed = 0.7;
	 			
	 			double deltaX;
	 			double deltaY;
	 			double setupangle = 0.0;
	 			
	 			switch(m_autonomousCase)
	 				{
	 				case 1:
	 					if(column < -90) m_navigate->m_position_x = startRampX;
	 					else m_navigate->m_position_x = startZoneFarX;
	 					m_navigate->m_position_y = column;
	 					m_navigate->m_destination_x = farRow1X + 5;
	 					deltaX = farRow1X - goalX;
	 					deltaY = column - goalY;
	 					setupangle = atan2(deltaY,deltaX);
	 					m_navigate->m_destination_y = column + 5*tan(setupangle);
	 					AutonomousKicker(0,1.56);
	 					m_robotDrive->TankDrive(0.0,0.0);
	 					if((m_autoDelay + 0.5) * LOOPSPERSEC < m_autoPeriodicLoops) m_autonomousCase = 2;
	 					break;
	 				case 2:
	 					AutonomousKicker(0,1.56);
	 					if(kDriveToDestination()) m_autonomousCase = 3;
	 					break;
	 				case 3:
	 					AutonomousKicker(0,1.56);
	 					m_navigate->m_destination_x = farRow1X;
	 					m_navigate->m_destination_y = column;
	 					m_autonomousCase = 4;
	 					break;
	 				case 4:
	 					AutonomousKicker(0,1.56);
	 					if(kDriveToDestination()) m_autonomousCase = 5;
	 					break;
	 				case 5:
	 					AutonomousKicker(1,1.56);
	 					m_robotDrive->TankDrive(0.5,0.5);
	 					m_autonomousCase = 6;
	 					break;
	 				case 6:
	 					m_robotDrive->TankDrive(0.5,0.5);
	 					if (AutonomousKicker(0,1.56)) m_autonomousCase = 7;
	 					break;
	 				case 7:
	 					m_navigate->m_destination_x = farRow2X + 5;
	 					deltaX = farRow2X - goalX;
	 					deltaY = column - goalY;
	 					setupangle = atan2(deltaY,deltaX);
	 					m_navigate->m_destination_y = column + 5*tan(setupangle);
	 					AutonomousKicker(0,1.7);
	 					m_robotDrive->TankDrive(0.0,0.0);
	 					m_autonomousCase = 8;
	 					break;
	 				case 8:
	 					AutonomousKicker(0,1.7);
	 					if(kDriveToDestination()) m_autonomousCase = 9;
	 					break;
	 				case 9:
	 					AutonomousKicker(0,1.7);
	 					m_navigate->m_destination_x = farRow2X;
	 					m_navigate->m_destination_y = column;
	 					m_autonomousCase = 10;
	 					break;
	 				case 10:
	 					AutonomousKicker(0,1.7);
	 					if(kDriveToDestination()) m_autonomousCase = 11;
	 					break;
	 				case 11:
	 					AutonomousKicker(1,1.7);
	 					m_robotDrive->TankDrive(0.5,0.5);
	 					m_autonomousCase = 12;
	 					break;
	 				case 12:
	 					m_robotDrive->TankDrive(0.5,0.5);
	 					if (AutonomousKicker(0,1.7)) m_autonomousCase = 13;
	 					break;
	 				case 13:
	 					m_navigate->m_destination_x = farRow3X + 5;
	 					deltaX = farRow3X - goalX;
	 					deltaY = column - goalY;
	 					setupangle = atan2(deltaY,deltaX);
	 					m_navigate->m_destination_y = column + 5*tan(setupangle);
	 					AutonomousKicker(0,1.8);
	 					m_robotDrive->TankDrive(0.0,0.0);
	 					m_autonomousCase = 14;
	 					break;
	 				case 14:
	 					AutonomousKicker(0,1.8);
	 					if(kDriveToDestination()) m_autonomousCase = 15;
	 					break;
	 				case 15:
	 					AutonomousKicker(0,1.8);
	 					m_navigate->m_destination_x = farRow3X;
	 					m_navigate->m_destination_y = column;
	 					m_autonomousCase = 16;
	 					break;
	 				case 16:
	 					AutonomousKicker(0,1.8);
	 					if(kDriveToDestination()) m_autonomousCase = 17;
	 					break;
	 				case 17:
	 					AutonomousKicker(1,1.8);
	 					m_robotDrive->TankDrive(0.5,0.5);
	 					m_autonomousCase = 18;
	 					break;
	 				case 18:
	 					m_robotDrive->TankDrive(0.5,0.5);
	 					if (AutonomousKicker(0,1.8)) m_autonomousCase = 33;
	 					break;
	 				case 33:
	 					m_robotDrive->TankDrive(0.0,0.0);
	 					AutonomousKicker(0,3.0);
	 					break;
	 				}
	 		}
};

START_ROBOT_CLASS(HotBot2010);
