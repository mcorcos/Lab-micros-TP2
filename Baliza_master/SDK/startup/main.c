/***************************************************************************//**
  @file     main.c
  @brief    FW main
  @author   Nicolás Magliola
 ******************************************************************************/

#include "hardware.h"

void App_Init (void);
void App_Run (void);


int main (void)
{
    hw_Init();
    hw_EnableInterrupts();
    App_Init(); /* Program-specific setup */


    __FOREVER__
         App_Run(); /* Program-specific loop  */
}
