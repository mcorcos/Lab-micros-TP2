/***************************************************************************//**
  @file     SysTick.h
  @brief    SysTick driver
  @author   grupo4
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/


#include "SysTick.h"



/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define DEVELOPMENT_MODE    1

#define SYSTICK_LOAD_INIT   ((__CORE_CLOCK__/SYSTICK_ISR_FREQUENCY_HZ) - 1U)

#if SYSTICK_LOAD_INIT > (1<<24)
#error Overflow de SysTick! Ajustar  __CORE_CLOCK__ y SYSTICK_ISR_FREQUENCY_HZ!
#endif // SYSTICK_LOAD_INIT > (1<<24)


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/
void (*ptr_to_fun_systick)(void)=0;
/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/


/**
 * @brief Initialize SysTic driver
 * @param funcallback Function to be call every SysTick
 * @return Initialization and registration succeed
 */
bool SysTick_Init (void (*funcallback)(void))
{
    static bool yaInit = false;
#if DEVELOPMENT_MODE
    if (!yaInit && funcallback)
#endif // DEVELOPMENT_MODE
    {
        SysTick->CTRL = 0x00;
        SysTick->LOAD =SYSTICK_LOAD_INIT; //100000-1;//
        SysTick->VAL  = 0x00;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

        ptr_to_fun_systick = funcallback;
        yaInit = true;
    }
    return yaInit;
}


__ISR__ SysTick_Handler (void)
{
#if DEVELOPMENT_MODE
    if (ptr_to_fun_systick)
#endif // DEVELOPMENT_MODE
    {
    	ptr_to_fun_systick();
    }
}



/*******************************************************************************
 ******************************************************************************/
