/***************************************************************************//**
  @file     timer.c
  @brief    Timer driver. Advance implementation
  @author   Nicolás Magliola
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "timer.h"

#include "SysTick.h"


/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#if TIMER_TICK_MS != (1000U/SYSTICK_ISR_FREQUENCY_HZ)
#error Las frecuencias no coinciden!!
#endif // TIMER_TICK_MS != (1000U/SYSTICK_ISR_FREQUENCY_HZ)

#define TIMER_DEVELOPMENT_MODE    0

#define TIMER_ID_INTERNAL   0


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef struct {
	ttick_t             period;
	ttick_t             cnt;
    tim_callback_t      callback;
    uint8_t             mode        : 1;
    uint8_t             running     : 1;
    uint8_t             expired     : 1;
    uint8_t             unused      : 5;
} timer_t;


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Periodic service
 */
static void timer_isr(void);


/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static timer_t timers[TIMERS_MAX_CANT];
static tim_id_t timers_cant = TIMER_ID_INTERNAL+1;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void timerInit(void)
{
    static bool yaInit = false;
    if (yaInit)
        return;

    SysTick_Init(timer_isr); // init peripheral

    yaInit = true;
}


tim_id_t timerGetId(void)
{
#ifdef TIMER_DEVELOPMENT_MODE
    if (timers_cant >= TIMERS_MAX_CANT)
    {
        return TIMER_INVALID_ID;
    }
    else
#endif // TIMER_DEVELOPMENT_MODE
    {
    	timers[timers_cant].running = false;
    	timers[timers_cant].expired = 0;
        return timers_cant++;
    }
}


void timerStart(tim_id_t id, ttick_t ticks, uint8_t mode, tim_callback_t callback)
{
#ifdef TIMER_DEVELOPMENT_MODE
    if ((id < timers_cant) && (mode < CANT_TIM_MODES))
#endif // TIMER_DEVELOPMENT_MODE
    {


    	timer_t new_timer;
    	bool was_running[TIMERS_MAX_CANT]={false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};

    	for(tim_id_t i=0;i<timers_cant;i++)
    	{
    		if(timers[i].running)
    		{
    			was_running[i]=true;
    		}
    		timers[i].running = false;
    	}

    	new_timer.period = ticks;
    	new_timer.cnt = ticks;
    	new_timer.callback = callback;
    	new_timer.mode = mode;
    	new_timer.expired = 0;

    	timers[id] = new_timer;
    	for(tim_id_t i=0;i<timers_cant;i++)
		{
    		if(was_running[i] || id==i)
    			timers[i].running = true;
		}
    }
}


void timerStop(tim_id_t id)
{

	timers[id].running = false;
	timers[id].cnt = timers[id].period;
	timers[id].expired = false;
}


bool timerExpired(tim_id_t id)
{

	if(timers[id].expired)
	{
		timers[id].expired = 0;
		return true;
	}
	else return false;
}

bool timerRunning(tim_id_t id)
{
	return timers[id].running;
}


void timerDelay(ttick_t ticks)
{
    timerStart(TIMER_ID_INTERNAL, ticks, TIM_MODE_SINGLESHOT, NULL);
    while (!timerExpired(TIMER_ID_INTERNAL))
    {
        // wait...
    }
}


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void timer_isr(void)
{
    // ****** COMPLETAR ******
    // decremento los timers activos
    // si hubo timeout!
    // 1) execute action: callback or set flag
    // 2) update state
	for(tim_id_t i = 0; i<timers_cant; i++)
	{
		if(timers[i].running)
		{
			timers[i].cnt--;
			if(!timers[i].cnt)
			{
				if(timers[i].callback != NULL) timers[i].callback();
				if(timers[i].mode == TIM_MODE_SINGLESHOT)
				{
					timers[i].running = false;
				}
				else if(timers[i].mode == TIM_MODE_PERIODIC)
				{
					timers[i].cnt = timers[i].period;
				}
				timers[i].expired = true;	//aunque sea periodico levanta el flag de expired para avisar que termino al menos una vez

			}
		}
	}
}


/******************************************************************************/
