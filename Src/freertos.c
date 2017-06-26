/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************
*/


/**
* @author Murat Demirtas <muratdemirtastr@gmail.com>
* @brief  Os Message Handling Example for interrupt service routines
* @license GNU General Public License (version 2)
* @note Total FreeRTOS Heap Size is 15KB.
*/
#include "FreeRTOS.h"		/*used for FreeRTOS Backend*/
#include "task.h"			/*used for FreeRTOS Tasks*/
#include "cmsis_os.h"	    /*used for Cortex Standarts*/
#include "stdio.h"			/*used for printf function*/
#include "stm32f4xx_hal.h"  /*used for STM HAL Library Driver*/
#include "stm32f4xx.h"		/*used for STM Backend Driver*/
#include "stm32f4xx_it.h"   /*Used for gpio interrupt callback*/
#include <stdarg.h>		    /*Used for combine string and arguments*/
#include "stdbool.h"		/*Used for boolean variables*/

osThreadId defaultTaskHandle;	/*FreeRTOS standart Task handler*/
osThreadId myTask02Handle;		/*Example task handler*/

/*OS Message Queues for communication over threads and isr()*/
osMessageQId interruptMessage;
osMessageQId taskMessage;
osMessageQId tasktoTaskMessage;

/*OS Semaphore for print debug messages*/
osSemaphoreId printerSemphrHandle;
osSemaphoreDef(printerSemphr);

/*function prototype for debugging*/
void printDebugMessage(bool newline, const char* format, ...);

/*function prototype for starting tasks*/
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/*function prototype for init FreeRTOS settings*/
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
* @brief This function will prepare FreeRTOS settings.
*/
void MX_FREERTOS_Init(void) {
	
	/*create semaphore*/
	printerSemphrHandle = osSemaphoreCreate(osSemaphore(printerSemphr), 1);
	
	/*define standart default task with minimal stack size*/
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/*define example task with minimal stack size*/
	osThreadDef(myTask02, StartTask02, osPriorityHigh, 0, 128);
	myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

	/*define interrupt message queue*/
	osMessageQDef(myQueue01, 16, uint32_t);
	interruptMessage = osMessageCreate(osMessageQ(myQueue01), NULL);

	/*define task message queue*/
	osMessageQDef(myQueue02, 16, uint32_t);
	taskMessage = osMessageCreate(osMessageQ(myQueue02), NULL);
	
	/*define tasktoTaskMessage queue*/
	osMessageQDef(myQueue03, 16, uint32_t);
	tasktoTaskMessage = osMessageCreate(osMessageQ(myQueue03), NULL);
	
	/*try to take print semaphore on startup*/
	if (osSemaphoreWait(printerSemphrHandle, osWaitForever) == osOK)
	{
		printf("printf semaphore initialized and working with no error\n");
		osSemaphoreRelease(printerSemphrHandle);
	}
		
	else
		printf("printf semaphore doesn't work,reset or contiune?\n");
	
	/*try to work message queues if they have not error*/
	osEvent tempEvent;
	char* temp_msg = "null";
	
	/*put example messages to queues*/
	osMessagePut(interruptMessage, (uint32_t)temp_msg, osWaitForever);
	osMessagePut(taskMessage, (uint32_t)temp_msg, osWaitForever);
	osMessagePut(tasktoTaskMessage, (uint32_t)temp_msg, osWaitForever);
	
	/*try to receive them*/
	tempEvent = osMessageGet(interruptMessage, osWaitForever);
	
	if (tempEvent.status == osEventMessage)
		printDebugMessage(true, "interrupt message handler working with no error");
		
	tempEvent = osMessageGet(taskMessage, osWaitForever);
	
	if (tempEvent.status == osEventMessage)
		printDebugMessage(true, "task message handler working with no error");
	
	tempEvent = osMessageGet(tasktoTaskMessage, osWaitForever);
	
	if (tempEvent.status == osEventMessage)
		printDebugMessage(true, "task to task message handler working with no error");
	
	/*free temp message*/
	temp_msg = NULL;
}



/**
* @brief  printf based function for printing debug message
* @param  newline and debug message with arguments
* @return 
*/
void printDebugMessage(bool newline, const char* format, ...)
{
	/*try to take printer semaphore*/
	if (osSemaphoreWait(printerSemphrHandle, osWaitForever) == osOK)
	{
		/*we must combine standart message and arguments*/
		
		/*create buffer for debugmessage*/
		char message[300];       
		/*start stdarg functions for combine strings and arguments*/
		va_list args; va_start(args, format);
		/*combine it*/
		vsnprintf(message, sizeof(message), format, args);
		printf("%s",message);   /*now you can print debug message*/
		va_end(args);

		/*if you want newline then printf newline escape character*/
		if (newline)
			printf("\n");

		/*release semaphores for another calls*/
		osSemaphoreRelease(printerSemphrHandle);
		return;
	}
	/*if error occurred,notify us with standart printf*/
	else
	{
		printf("printer semaphore take failed, is your program running good?\n");
	}
	
	return;
}

/**
* @brief  This is FreeRTOS task
* @param  parameters as thread start arguments as (String[] args)
* @return this function must be not return anyway
*/
void StartTask02(void const * argument)
{
	/*you can declare your variables for this task*/
	long int errorCount = 0;;
	/*operating system event for receiving os events from rtos kernel*/
	osEvent taskMessageHandler;
	
	/*infinite loop of this thread*/
	for (;;)
	{
		osDelay(1);  /*wait 1 ms for stability*/
		/*wait until kernel message*/
		taskMessageHandler = osMessageGet(taskMessage, osWaitForever);
		
		/*if we are here, then receive message*/
		if (taskMessageHandler.status == osEventMessage)
		{
			/*create task message*/
			char* messagetoInterrupt = "[TASK]hello isr(), i received your message thank you";
			/*receive message as char pointer*/
			//printDebugMessage(true, "[TASK]Received Message is %s", taskMessageHandler.value.p);
			
			/*try to send message to isr() function*/
			if (osMessagePut(interruptMessage, (uint32_t)messagetoInterrupt, osWaitForever) != osOK)
				errorCount = errorCount + 1;
			
			/*try to send message to another thread*/
			if (osMessagePut(tasktoTaskMessage, (uint32_t)taskMessageHandler.value.p, osWaitForever) != osOK)
				errorCount = errorCount + 1;
		
			/*if error, then notify us*/
			else
				printDebugMessage(true, "[TASK]Message can not send to isr(),is queue full or what?");
			
		}
	}
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
* @param parameter is only GPIO interrupt pin
* @return none
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*notify us when button is pressed*/
	printDebugMessage(true, 
		"[ISR]Pressed interrupt button, now we entering interrupt service routine callback");
	/*clear interrup flag of GPIO pin, 
				this will re enable by HAL library after exit this function*/
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

	/*creating os event handler*/
	osEvent messageHandler;
	/*create task message*/
	char * messagetotask = "[ISR]hello, i'm sending message to you from isr(), are u fine?";
	
	/*try to send message to kernel*/
	if (osMessagePut(taskMessage, (uint32_t)messagetotask, osWaitForever) == osOK)
		printDebugMessage(true, "[ISR]Message was send succesfully to task");
	else
		printDebugMessage(true, "[ISR]Message can not send to task, is queue full or what?");
			
	/*wait task message until kernel response*/
	messageHandler = osMessageGet(interruptMessage, osWaitForever);
	/*if we are here, then we have got os message. print them*/
	printDebugMessage(true, "[ISR]Message Received from, %s", messageHandler.value.p);
	return;
		
}

/**
* @brief This function is default FreeRTOS Task
* @param parameters as thread start arguments as (String[] args)
* @return this function must be not return anyway
*/
void StartDefaultTask(void const * argument)
{
	osEvent taskToTaskEvent;   /*operating system event for receiving os events from rtos kernel*/
	
	/*you can declare your variables for this task*/
	int m_errorCount = 0;
	
	/*infinite loop of this thread*/
	for (;;)
	{
		osDelay(1);		/*delay 1 ms for stability*/
		
		/*wait forever until when kernel send event message*/
		taskToTaskEvent = osMessageGet(tasktoTaskMessage, osWaitForever);
		
		/*if we are here, then we received event */
		if (taskToTaskEvent.status == osEventMessage)
		{
			/*if we have message then */
			char * receivedMessage = taskToTaskEvent.value.p;  /*get message as char pointer*/
	
			/*process this data, do what if you want*/
		}	
	}
}
