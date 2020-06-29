// Make sure tick rate is set to at least 10kHz
// This is probably super high according to the FreeRTOS
// docs, but it is because the rotary encoder doesnt function
// well at low polling rates.  If this take rate has too much
// overhead, then a new encoder will be implemented in
// hardware and the 1ms task will be moved to 10ms.
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "motor_encoder_controller.h"
#include "nexys4IO.h"

#include "display.h"
#include "input.h"
#include "pid_cont.h"

#include "xwdttb.h"

#define NX4IO_BADDR	XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define MEC_BADDR XPAR_MOTOR_ENCODER_CONTRO_0_S00_AXI_BASEADDR

#define WDT_INTERRUPT_ID	XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR


bool wdt_1ms = 0;
bool wdt_2ms = 0;
bool wdt_100ms = 0;

typedef struct INPUTS_HANDOFF_STRUCT{
	EncoderState rotenc;
	uint16_t rpm_sp;
	uint16_t rpm_meas;
    bool motor_kill;
} InputsHandoff;

volatile QueueHandle_t InputsHandoffQueue;

typedef struct CONTROL_HANDOFF_STRUCT{
    uint16_t kp_fp1000;
    uint16_t ki_fp1000;
    uint16_t kd_fp1000;
} ControlHandoff;

volatile QueueHandle_t ControlHandoffQueue;


xSemaphoreHandle motor_dir_change_rq_sem;

void v1msTasks(void * pvParameters);
void v20msTasks(void * pvParameters);
void v100msTasks(void * pvParameters);

void vApplicationIdleHook( void ) {
	//xil_printf("Idling: %d\n", xTaskGetTickCountFromISR());
    return;
}

void vApplicationTickHook( void ) {
    //xil_printf("Ticking\n");
    return;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName) {
    xil_printf("%s overflowed!\n", pcTaskName);
    return;
}

int WDT_initialize(void);
void WDT_Handler(void *pvUnused);
XWdtTb WDT; // Watch dog timer instance

// Watch Dog configure
int WDT_initialize(void)
{
    XWdtTb_Config *configure;

    // Configure WDT
    configure = XWdtTb_LookupConfig(XPAR_WDTTB_0_DEVICE_ID);   

    XWdtTb_CfgInitialize(&WDT, configure, configure->BaseAddr); 

    XWdtTb_ProgramWDTWidth(&WDT, 28);

    xPortInstallInterruptHandler(WDT_INTERRUPT_ID, WDT_Handler, NULL);

    vPortEnableInterrupt(WDT_INTERRUPT_ID);
    
    XWdtTb_Start(&WDT);

    return XST_SUCCESS;
}



//  Watchdog timer handler
void WDT_Handler(void *pvUnused) {
    
    SwitchInputs switch_in = get_switch_inputs();
    //xil_printf("wdt100 %d, wdt 1 %d, wdt20 %d\n", wdt_100ms, wdt_1ms, wdt_2ms);
    if( (switch_in.raw_state & 0x8000) || (!wdt_100ms) || (!wdt_1ms) || (!wdt_2ms) ) {  
        for(;;) {
        xil_printf("Crashed......\n");
        NX4IO_setLEDs(0x5555);
        vTaskEndScheduler();           // Ends Scheduler
        
        }  
    }
    else 
    {
        XWdtTb_RestartWdt(&WDT);              // restarts WDT
    }

    wdt_2ms = 0;
    wdt_1ms = 0;
    wdt_100ms = 0;
    return;
}

int main(void)
{
    XStatus sts;
    BaseType_t ret;

	// Announcement
	xil_printf("Hello from Project 2!\r\n");
    xil_printf("R. Holt and D. Parmar\r\n");

        // General Setup
	sts = NX4IO_initialize(NX4IO_BADDR);
	if(sts == XST_FAILURE) {
		xil_printf("Couldnt initialize NX4IO ... Exiting.\n");
		return -1;	
	}

    setup_displays();
	setup_inputs();
    
	//Initialize the platform
    init_platform();

    sts = MotorEncCont_initialize(MEC_BADDR);
	if(sts == XST_FAILURE) {
		xil_printf("Couldnt initialize Motor Controller ... Exiting.\n");
		return -1;
	}

    // Set the output shaft ratio for the motor.  A value of 9.71 means
    // the output shaft is spinning 9.71 times slower than the input shaft.
    MotorEncCont_set_output_ratio(9.71);

    // Create Queue which faster sampling inputs will
    // use to hand off to the control algorithm.
    // Only need a size of one because only the latest
    // inputs really matter.  Input handling tasks will
    // overwrite and control algorithm will only peek.
    InputsHandoffQueue = xQueueCreate(1, sizeof(InputsHandoff));
    xil_printf("Post queue 1: %d bytes\n",xPortGetFreeHeapSize());

    // Create Queue which faster sampling control algorithm will
    // use to hand off to the display algorithms.
    // Only need a size of one because only the latest
    // control outputs really matter.  Control task will
    // overwrite and display tasks will only peek.
    ControlHandoffQueue = xQueueCreate(1, sizeof(ControlHandoff)); 
    xil_printf("Post queue 2: %d bytes\n",xPortGetFreeHeapSize());

    motor_dir_change_rq_sem = xSemaphoreCreateBinary();

	//Create 1ms task
	ret =  xTaskCreate(v1msTasks,
				    "1msTask",
				    2048,
				    NULL,
				    3,
				    NULL);
	if(ret != pdPASS) {
        xil_printf("1ms task allocation failed\n");
    }
    xil_printf("Post 1ms task: %d bytes\n",xPortGetFreeHeapSize());
	
    //Create 20ms task
	ret = xTaskCreate(v20msTasks,
				"20msTask",
				2048,
				NULL,
				2,
				NULL);

    if(ret != pdPASS) 
    {
        xil_printf("20ms task allocation failed\n");
    }
    xil_printf("Post 20ms task: %d bytes\n",xPortGetFreeHeapSize());

    //Create 100ms task
	ret = xTaskCreate(v100msTasks,
				"100msTask",
				2048,
				NULL,
				1,
				NULL);
    if(ret != pdPASS) {
        xil_printf("100ms task allocation failed\n");
    }
    xil_printf("Post 100ms task: %d bytes\n",xPortGetFreeHeapSize());

    // Initialize Watch Dog Timer
    sts = WDT_initialize();
    if(sts == XST_FAILURE) {
		xil_printf("Couldnt initialize WatchDog Timer ... Exiting.\n");
		return -1;	
	}

    xil_printf("Starting scheduler with %d tasks.\n", uxTaskGetNumberOfTasks());
	vTaskStartScheduler();
    xil_printf("Ending!");
    //cleanup_platform();
	return -1;
}


// Perform an action every 1 ms.
void v1msTasks(void * pvParameters)
{
    TickType_t xLastWakeTime;

    // Setup input variables
    uint16_t rpm_meas;
    uint16_t rpm_sp = 200;
    bool motor_kill;
    EncoderState rotenc;
    PushButtons pbs;
   
    xil_printf("Setting up %s\n", pcTaskGetName(NULL));
    // Name is confusing, its the struct that holds the
    // outputs of this 1ms input handling task
    InputsHandoff inputs_output;
    
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        // Reading the Switch Values
        SwitchInputs current_sw_value = get_switch_inputs();

        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, 10);

        rotenc = get_enc_state();
        pbs = get_push_buttons();

	    // update RPM states based on inputs
        rpm_meas = MotorEncCont_get_rpm();
	    if(pbs.center) {
            rpm_sp = 0;
            motor_kill = true;
        } else {
            //rpm_sp += rotenc.rot_dir;
	        rpm_sp +=  update_motor_speed_sw( &current_sw_value, &rotenc);
            motor_kill = false;
        }

        // If the encoder requests a direction change, then give a semaphore to indicate
        // to the 20ms task to change the direction.
        // This information can't be passed in the queue because of the high likelihood
        // that it is overwritten prior to the 20ms task getting it. 
        if(rotenc.switch_dir_change != 0) {
            xSemaphoreGive(motor_dir_change_rq_sem);
        }

        // Put the inputs into the queue for the PID controller
        // to use
        inputs_output.rotenc = rotenc;
        inputs_output.rpm_sp = rpm_sp;
        inputs_output.rpm_meas = rpm_meas;
        inputs_output.motor_kill = motor_kill;
        xQueueOverwrite(InputsHandoffQueue, &inputs_output);

        // Set a flag to indicate to the Wdt that things are going along fine
        wdt_1ms = 1;
    }

    // Better not get here, if so, at least call delete to clean up
    vTaskDelete(NULL);
    return;
}

// Perform every 20 ms.
void v20msTasks(void * pvParameters)
{
    //xil_printf("%d",uxTaskGetStackHighWaterMark(NULL));
    TickType_t xLastWakeTime;
    
    xil_printf("Setting up %s\n", pcTaskGetName(NULL));

    // Initialize control algorithm variables
    InputsHandoff control_inputs;
    ControlHandoff control_outputs = {(uint16_t) 0.01 * 1000,
                                      (uint16_t) 0.01 * 1000,
                                      (uint16_t) 0.01 * 1000};
    int16_t rpm_err;
    uint8_t pid_cur_dc;

    //Setup PID with pre-tuned gains.
	pid_init(0.0, 0.308, 0.441, 0.054, 0.02, 255.0, 1.0);
	pid_enable_gains(true, true, true);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    
    // Wait up to one 20ms task cycletime for the inputs to populate 
    if(xQueuePeek(InputsHandoffQueue, &control_inputs, 200) == pdFALSE) {
        xil_printf("%s: Couldnt get initial control inputs.  Something is wrong\n", pcTaskGetName(NULL));
        vTaskDelete(NULL);
    }

    for( ;; )
    {
        
        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, 200);
        // Perform action here.
        //xil_printf("Running %s\n", pcTaskGetName(NULL));

        if(xQueuePeek(InputsHandoffQueue, &control_inputs, 0) == pdFALSE) {
            xil_printf("%s: Control values not being received\n", pcTaskGetName(NULL));
        }

        // Reading the switch values
        SwitchInputs current_sw_value = get_switch_inputs();

        // Reading the push buttons
        PushButtons current_pb_value = get_push_buttons();

        rpm_err = control_inputs.rpm_sp - control_inputs.rpm_meas;
        
        // Slecting the PID Parameter to change along with the gain value
        update_pid_value_sw( &current_sw_value, &current_pb_value);
        
        if(control_inputs.motor_kill) {
            MotorEncCont_set_dutycycle(0);
        }
        else 
        {
	        pid_cur_dc = (uint8_t) pid_perform_step(control_inputs.rpm_sp, control_inputs.rpm_meas);
	        
            //if(control_inputs.rotenc.switch_dir_change != 0) {
	        if(pdTRUE == xSemaphoreTake(motor_dir_change_rq_sem, 0)) 
            {
            	xil_printf("Detected change in rotary encoder switch direction.\n");
	        	MotorEncCont_set_dutycycle_pct(0);
	        	//sleep 2s
	        	wdt_2ms = 1;
	        	vTaskDelay(20000);
	        	xQueuePeek(InputsHandoffQueue, &control_inputs, 0);
                xil_printf("Changing direction to %d\n", control_inputs.rotenc.switch_up);
	        	MotorEncCont_set_direction(control_inputs.rotenc.switch_up);
	        }

	        // Set the new duty cycle as determined by the PID controller
	        MotorEncCont_set_dutycycle(pid_cur_dc);
        }
	    xil_printf("%d, %d, %d, %d\n", control_inputs.rpm_meas, control_inputs.rpm_sp, pid_cur_dc, rpm_err);

        control_outputs.kp_fp1000 = pid_get_gain_u16fp(P);
        control_outputs.ki_fp1000 = pid_get_gain_u16fp(I);
        control_outputs.kd_fp1000 = pid_get_gain_u16fp(D);
    
        // Update the 7 segs
	    disp_meas_speed(control_inputs.rpm_meas);
	    disp_set_speed(control_inputs.rpm_sp);

        // Update the used LEDs
        set_disp_gains_applied_fb_leds(get_enabled_gains_bitfield());

        // Put the output gains in the queue
        xQueueOverwrite(ControlHandoffQueue, &control_outputs);
        wdt_2ms = 1;
    }

    // Better not get here, if so, at least call delete to clean up
    vTaskDelete(NULL);
    return;
}

// Perform every 100 ms.
void v100msTasks(void * pvParameters)
{
    TickType_t xLastWakeTime;

    ControlHandoff disp_inputs_from_cont;
    InputsHandoff disp_inputs_from_inputs;
    
    xil_printf("Setting up %s\n", pcTaskGetName(NULL));

    set_oled_disp_pidgains_numeric(0xFFFF,
                                   0xFFFF,
                                   0xFFFF);

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Wait up to one 100ms task cycletime for the control outputs to populate 
    if(xQueuePeek(ControlHandoffQueue, &disp_inputs_from_cont, 1000) == pdFALSE) {
        xil_printf("%s: Couldnt get initial display inputs from control task.  Something is wrong\n", pcTaskGetName(NULL));
        vTaskDelete(NULL);
    }
    // Wait up to one 100ms task cycletime for the control outputs to populate
    if(xQueuePeek(InputsHandoffQueue, &disp_inputs_from_inputs, 1000) == pdFALSE) {
        xil_printf("%s: Couldnt get initial display inputs from input task.  Something is wrong\n", pcTaskGetName(NULL));
        vTaskDelete(NULL);
    }

    for( ;; )
    {
        // Wait for the next cycle.
        vTaskDelayUntil(&xLastWakeTime, 1000);
        //xil_printf("Running %s\n", pcTaskGetName(NULL));
        // Perform action here.

        // Poll the queue and copy data into disp_inputs
        if(xQueuePeek(ControlHandoffQueue, &disp_inputs_from_cont, 0) == pdFALSE) {
            xil_printf("%s: Control Handoff values not being received\n", pcTaskGetName(NULL));
        }
        // Poll the queue and copy data into disp_inputs
        if(xQueuePeek(InputsHandoffQueue, &disp_inputs_from_inputs, 0) == pdFALSE) {
            xil_printf("%s: Input Handoff values not being received\n", pcTaskGetName(NULL));
        }

        set_oled_disp_pidgains_numeric(disp_inputs_from_cont.kp_fp1000,
                                       disp_inputs_from_cont.ki_fp1000,
                                       disp_inputs_from_cont.kd_fp1000);

	    set_oled_disp_meas_mot_speed_numeric(disp_inputs_from_inputs.rpm_meas);
        wdt_100ms = 1;
    }

    // Better not get here, if so, at least call delete to clean up
    vTaskDelete(NULL);
    return;
}



