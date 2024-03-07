
#include "port_common.h"
#include "mbtimer.h"


eMBRcvState eRcvState;
volatile uint8_t mb_state_rtu_finish;
volatile uint16_t mb_timeout;
volatile uint16_t mb_downcounter;
extern TIM_HandleTypeDef htim3;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        if (!--mb_downcounter)
          xMBRTUTimerT35Expired();
    }
    
}

void xMBPortTimersInit( uint32_t usTim1Timerout50us)
{
    mb_timeout = usTim1Timerout50us;
}

void vMBPortTimersEnable( void )
{
    mb_downcounter = mb_timeout;
    HAL_TIM_Base_Start_IT(&htim3);
}

void vMBPortTimersDisable( void )
{
    HAL_TIM_Base_Stop_IT(&htim3);
}

void xMBRTUTimerT35Expired( void )
{
	switch ( eRcvState ) {
		/* Timer t35 expired. Startup phase is finished. */
		case STATE_RX_INIT:
			break;

		/* A frame was received and t35 expired. Notify the listener that
		* a new frame was received. */
		case STATE_RX_RCV:
			mb_state_rtu_finish = 1;
			break;
		
		/* An error occured while receiving the frame. */
		case STATE_RX_ERROR:
			break;

		/* Function called in an illegal state. */
		default:
			break;
	}
	vMBPortTimersDisable();
	eRcvState = STATE_RX_IDLE;

	//printf("tim3\r\n");
}
