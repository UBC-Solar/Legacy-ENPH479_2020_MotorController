/**
 *  @file state_machine.c
 *  @brief Main finite state machine for the motor controller
 *
 *  @date 2021/04/30
 *  @author Alex Ezzat (aezzat1)
 */

#define PI = 3.14
// Comment out the line below to disable printf debugging
#define PRINTF_DEBUG

#include "state_machine.h"

#ifdef PRINTF_DEBUG
#include "stdio.h"
#endif

/*============================================================================*/
/* PRIVATE FUNCTION PROTOTYPES */

// State functions:
void FSM_reset();
void FSM_readDriveCommand();
void FSM_torqueCommand();
void FSM_speedCommand();
void FSM_regenCommand();
void FSM_execRamp();
int FSM_checkFaults();
void FSM_fault();
void FSM_monitor();
void FSM_writeOutputs();

// Helper functions:
int convertUnits(int int16Val);
int checkMailbox();

#ifdef PRINTF_DEBUG
void printStatus(int print_ok);
void printMeasurements();
#endif

/*============================================================================*/
/* FSM STATE FUNCTIONS POINTER ARRAY */

// Function names go in this array declaration, and must be in the same order
// as the FSM_state_t enumeration.
void (* FSM_state_table[])()
    = {FSM_reset, FSM_readDriveCommand, FSM_torqueCommand, FSM_speedCommand, 
    		FSM_regenCommand, FSM_execRamp, FSM_checkFaults, FSM_fault, 
			FSM_monitor, FSM_writeOutputs};

/*============================================================================*/

/* FSM CONFIGURABLE CONSTANTS */
float WHEELRAD = 0.1; //Wheel (tire) radius, [m, float]
unsigned int NMAX = 4000; //Max motor speed, [RPM, int16_t]
unsigned int TMAX = 0.5; //Max motor torque, [Nm, int16_t]
unsigned int IBUSMAX = 10; //Max DC bus current, [A, int16_t]
unsigned int CARMASS = 1; //Vehicle mass, [kg, int16_t]
float VMAX = NMAX*(PI/30)*WHEELRAD //Max vehicle velocity, [m/s, int16_t]


/* FSM GLOBAL VARIABLES */
FSM_state_t FSM_state;
unsigned int last_uptime_tick; // for keeping track of uptime
unsigned int last_update_tick; // for control of measurement timing
int system_status; // status code for system

int lastCommandTime = 0; //Time of last CAN message received, [ms]
int TSL = 0; //Elapsed TimeSinceLast message, [ms]

/*============================================================================*/
/* FSM FUNCTIONS */

/**
 * @brief Initializes FSM state.
 *
 * This must be called once, before calling FSM_run().
 */
void FSM_init()
{
    last_uptime_tick = HAL_GetTick();
    system_status = 0;
    FSM_state = FSM_RESET;
#ifdef PRINTF_DEBUG
    printf("FSM initialized\n");
#endif
}

// This function should be placed in the main firmware loop
void FSM_run()
{
    unsigned int current_tick;

    FSM_state_table[FSM_state]();

    // Update uptime
    current_tick = HAL_GetTick();
    if (current_tick - last_uptime_tick >= 1000) {
        uptime++;
    }
}

/**
 * @brief Performs initialization of the motor controller.
 * This does not initialize the microcontroller. That must happen before running the
 * state machine.
 */
void FSM_reset()
{
    uptime = 0; // Reset uptime
    //CAN_initAll(); // Initialize CAN

#ifdef PRINTF_DEBUG
    printf("Controller initialized\n");
#endif

    last_update_tick = HAL_GetTick(); // initialize last_measurement_tick

#ifdef PRINTF_DEBUG
        printf("Entering NORMAL state\n");
#endif
        FSM_state = FSM_READDRIVECOMMAND;

    return;
}

/*
 * @brief Reads user drive commands from the CAN bus and determines required motor command
 */
void FSM_readDriveCommand()
{

	lastTime = HAL_GetTick(); //TO-DO: Check

	if (TSL > 250)
	{
		FSM_state = FSM_FAULT;
	}

	if (checkMailbox())
	{
		TSL = 0;
		//Read CAN
		// ISet = CAN.current //User current setpoint, [%, uint32]
		// VSet = CAN.speed //User speed setpoint, [m/s, uint32]
		ISet = convertType(ISet);
		VSet = convertType(Vset);

		if (ISet > 100)
			FSM_state = FSM_FAULT;
		else if (Iset == 0)
			FSM_state = FSM_TORQUECOMMAND;
		else if (Vset > VMax)
			FSM_state = FSM_TORQUECOMMAND;
		else if (Vset < VMax)
		{
			if (Vset == 0.0)
				FSM_state = FSM_REGENCOMMAND;
			else
				FSM_state = FSM_SPEEDCOMMAND;
		}
		else if (ISeet == 1.0)
			FSM_state = FSM_TORQUECOMMAND;
		else
			FSM_state = FSM_SPEEDCOMMAND;
	}
	else
	{
		dt = HAL_GetTick() - lastTime;
		TSL = TSL + dt;
	}

}

/*
 * @brief Writes a motor torque ramp
 */
void FSM_torqueCommand()
{
	//torque = conversion*TMax*ISet //Torque setpoint, [Nm, int16_t]
	MC_ProgramTorqueRampMotor1(TMax * ISet, 0);
}

/*
 * @brief Writes a motor speed ramp
 */
void FSM_speedCommand()
{

	MC_ProgramSpeedRampMotor1(VSet/6, rampTime);
}

/*
 * @brief Executes the set torque or speed ramp
 */
void FSM_execRamp()
{
	//If motor == stopped:
	//Motor start

	FSM_state = FSM_CHECKFAULTS;
}

/*
 * @brief Checks non-interrupt-based faults
 */
int FSM_checkFaults()
{

	FSM_state = FSM_MONITOR;
}

/*
 * @brief Disables the controller
 */
void FSM_fault()
{

}

/*
 * @brief
 */
void FSM_monitor()
{
	FSM_state = WRITEOUTPUTS;
}

/*
 * @brief Writes state variables to CAN bus
 */
void FSM_writeOutputs()
{

}

/*
 * State functions should follow this format:
 * void FSM_<state name>()
 * {
 *      <Things to do in this state no matter what>
 *
 *      if(<state change condition>)
 *      {
 *          <Things to do on state transition>
 *          FSM_state = <state to go to on next loop>;
 *      }
 *      else if (<other state change condition>)
 *      {
 *          <Things to do on state transition>
 *          FSM_state = <state to go to on next loop>;
 *      }
 *      // staying in the same state otherwise can be implicit.
 * }
 */

/*============================================================================*/
/* HELPER FUNCTIONS */

#ifdef PRINTF_DEBUG
/*
 * @brief Print FSM status
 */
void printStatus(int print_ok)
{

    return;
}

/*
 * @brief Print current controller values
 */
void printMeasurements()
{

}
#endif
