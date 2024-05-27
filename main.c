#include "cyhal.h"
#include "cybsp.h"
#include "cy_systick.h"
#include "cy_retarget_io.h"

#include <SSD1306.h>
#include <SSD1306_Commands.h> // INCLUDE FOR SSD1306 COMMAND MACRO'S AND THE GDDRAM BUFFER

#define SYS_TICK_MAX       0x00FFFFFF // Source: see Cy_SysTick_SetReload() https://infineon.github.io/psoc6pdl/pdl_api_reference_manual/html/group__group__systick__functions.html#gac600820b0201e4ad5a171e97f7642c62

#define PIN_JOYSTICK_X 	   P10_0
#define PIN_JOYSTICK_Y     P10_3
#define PIN_JOYSTICK_SW    P13_4
#define PIN_MOTOR_1_PWM    P12_0
#define PIN_MOTOR_1_SPEED  P9_4
#define ECHO_PIN 		   P9_0
#define TRIG_PIN 		   P9_1
#define PIN_LINE_DETECT    P9_2

#define PI                 3.141593f
#define WHEEL_DIAMETER     6.5f // CM
#define WHEEL_RADIUS       (WHEEL_DIAMETER / 2.0f)

#define TIMER_FREQ_TICKS   10000                           // 10KHz: 10Kticks/s => period: 100uS => TO DO: MORE SPECIFIC NAME WHICH TIMER IT IS!!!!!
#define TIMER_PERIOD_TICKS (2 / (1.0F / TIMER_FREQ_TICKS)) // 2s = 20Kticks @ 10KHz              => TO DO: MORE SPECIFIC NAME WHICH TIMER IT IS!!!!!
#define TIMER_PERIOD_TICKS_ECHO_TIMEOUT 299

#define PWM_FREQ_MOTOR1_HZ 60

#define HIGHEST_PRIORITY 2
#define ISR_GPIO_ULTRASONIC_ECHO_PRIORITY HIGHEST_PRIORITY // The sensor needs a higher priority than changing the motor pwm. When the ultrasonic detects an object that is to close it needs to stop the motor as soon as possible
#define ISR_PWM_MOTOR_PRIORITY 			  3

typedef struct Motor_State
{
	cyhal_pwm_t* pwm_obj;
	float pwm_duty_cycle; // motor speed
	bool is_emergency_break_triggerd;
} Motor_State;

typedef struct Ultrasonic_State
{
	float distance;
} Ultrasonic_State;

typedef struct SSD1306_State
{
	SSD1306_Drawing_state_t drawing_state;
} SSD1306_State;

typedef struct ADC_Motors_Duty_Cycle
{
	float pwm_duty_cycle_motor1;
	float pwm_duty_cycle_motor2;
} ADC_Motors_Duty_Cycle;

typedef struct ISR_GPIO_ultrasonic_echo_arg
{
	Ultrasonic_State* ultrasonic_state;
	Motor_State* motor_state;
	cyhal_timer_t* timer_obj_echo;
} ISR_GPIO_ultrasonic_echo_arg;

typedef struct ISR_TIMER_OLED_SSD1306_arg
{
	SSD1306_State* ssd1306_state;
} ISR_TIMER_OLED_SSD1306_arg;

void TIMER_ultrasonic_echo_start(cyhal_timer_t* timer_obj);
uint32_t TIMER_ultrasonic_echo_read(cyhal_timer_t* timer_obj);
void Ultrasonic_trigger_send(cyhal_timer_t* timer_obj);

void ISR_GPIO_joystick(void* handler_arg, cyhal_gpio_event_t event);
void ISR_TIMER_joystick(void* callback_arg, cyhal_timer_event_t event);
void ISR_GPIO_motor1_speed(void* handler_arg, cyhal_gpio_event_t event);
void ISR_GPIO_Line_Detect(void* handler_arg, cyhal_gpio_event_t event);
void ISR_PWM_motor1(void* callback_arg, cyhal_pwm_event_t event);
void ISR_GPIO_ultrasonic_echo(void* callback_arg, cyhal_gpio_event_t event);
void ISR_TIMER_ultrasonic_echo(void* callback_arg, cyhal_timer_event_t event);
void ISR_TIMER_OLED_SSD1306(void* callback_arg, cyhal_timer_event_t event);

 // void SSD1306_DrawDateTime(void); // TO DO: CREATE THIS FUNCTION  WITH AN RTC CLOCK


int main(void)
{
	// INITIALIZE BOARD FILES AND SERIAL COMMUNICATION
    cybsp_init(); // Initialize the device and board peripherals
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE); // Init serial communication

    // INITIALIZE GPIO'S
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1U);    // User LED output
    cyhal_gpio_init(PIN_JOYSTICK_SW, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, true);  // Joystick switch is internally connected to GND
    cyhal_gpio_init(PIN_MOTOR_1_SPEED, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, true);  // Motor1 speed input => QUESTION: THE SENSOR CAN ONLY PULL DOWN THE LINE, SO DO I NEED TO PULLUP THE LINE OR NOT ???
    cyhal_gpio_init(PIN_LINE_DETECT, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, true);  // Line detect input  => QUESTION: THE SENSOR CAN ONLY PULL DOWN THE LINE, SO DO I NEED TO PULLUP THE LINE OR NOT ???

    // ENABLE GLOBAL INTERRUPTS
    __enable_irq(); // Enable global interrupts

    // MAIN APPLICATION CLOCK
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, SYS_TICK_MAX); // Init the CPU CLK timer
    Cy_SysTick_Enable();                                            // Enable the CPU timer


    // JOYSTICK
	// INITIALIZE TIMER: WHEN THE JOYSTICK BUTTON IS PRESSED AN LED WILL LIGHT UP, START A ONE-SHOT TIMER THAT TURNS THE LED OFF IF THE JOYSTICK ISN'T PRESSED IN TIME, WHEN PRESSED IN TIME, LEAVE THE LED ON AND START THE TIMER AGAIN.
	cyhal_timer_t timer_obj_joystick;
	const cyhal_timer_cfg_t timer_cfg_joystick =
	{
		.compare_value = 0,                    // Timer compare value, not used
		.period        = TIMER_PERIOD_TICKS,   // Define the timer time-out period: 2s
		.direction     = CYHAL_TIMER_DIR_UP,   // Timer counts up
		.is_compare    = false,                // Don't use compare mode
		.is_continuous = false,                // One-shot timer
		.value         = 0                     // Initial value of counter
	};

	cyhal_timer_init(&timer_obj_joystick, NC, NULL); // Initialize the timer object. Does not use pin output ('pin' is NC) and does not use a pre-configured clock source ('clk' is NULL).
	cyhal_timer_configure(&timer_obj_joystick, &timer_cfg_joystick); // Apply timer configuration such as period, count direction, run mode, etc.
	cyhal_timer_set_frequency(&timer_obj_joystick, TIMER_FREQ_TICKS);
	cyhal_timer_register_callback(&timer_obj_joystick, ISR_TIMER_joystick, NULL);
	cyhal_timer_enable_event(&timer_obj_joystick, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

	// GPIO: REGISTER JOYSTICK SWITCH CALLBACK AND ENABLE THE CALLBACK EVENT
	cyhal_gpio_callback_data_t gpio_cb_data_joystick =
	{
		.callback     = ISR_GPIO_joystick,
		.callback_arg = (void*)&timer_obj_joystick // Pass the timer object to the joystick switch interrupt so that it is able to start the timer
	};
	cyhal_gpio_register_callback(PIN_JOYSTICK_SW, &gpio_cb_data_joystick);
	cyhal_gpio_enable_event(PIN_JOYSTICK_SW, CYHAL_GPIO_IRQ_FALL, 3, true); // When the pin is pulled low, the joystick switch is pressed

	// INITIALIZE ADC WITH TWO CHANNELS, ONE CHANNEL FOR THE JOYSITCK X POSITION AND ONE FOR THE JOYSTICK Y POSITION
    cyhal_adc_t         adc_obj_0;
	cyhal_adc_channel_t adc_chan_0_obj, adc_chan_1_obj;
	const cyhal_adc_channel_config_t channel_0_config =
		{ .enable_averaging = false, .min_acquisition_ns = 220, .enabled = true };

	cyhal_adc_init(&adc_obj_0, PIN_JOYSTICK_X, NULL); // Initialize ADC block. => Documentation The pin number doesn't really get an ADC assigned
	cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj_0, PIN_JOYSTICK_X, CYHAL_ADC_VNEG, &channel_0_config); // Initialize ADC channel 0 on the ADC block adc_obj_0 => This time the channel will be coupled with the pin number
	cyhal_adc_channel_init_diff(&adc_chan_1_obj, &adc_obj_0, PIN_JOYSTICK_Y, CYHAL_ADC_VNEG, &channel_0_config); // Initialize ADC channel 1 on the ADC block adc_obj_0

	// DONT'T RELEASE THE ADC OBJ BECAUSE IT IS NEEDED IN THE FOR LOOP!
	// Release ADC and channel objects when no longer needed
	// cyhal_adc_channel_free(&adc_chan_0_obj);
	// cyhal_adc_free(&adc_obj);

	// LINE DETECTOR
	cyhal_gpio_callback_data_t gpio_cb_data_line_detector =
	{
		.callback     = ISR_GPIO_Line_Detect
	};
	cyhal_gpio_register_callback(PIN_LINE_DETECT, &gpio_cb_data_line_detector);
	cyhal_gpio_enable_event(PIN_LINE_DETECT, CYHAL_GPIO_IRQ_BOTH, 3, true);

	// MOTOR 1
	// INITIALIZE THE PWM SIGNALS AND INTERRUPTS TO CONTROL THE MOTOR SPEED => TESTING WITH AN LED FOR NOW!
	cyhal_pwm_t pwm_obj_motor1;
	Motor_State motor1_state = { .pwm_obj = &pwm_obj_motor1, .pwm_duty_cycle = 0.0f, .is_emergency_break_triggerd = false };

	cyhal_pwm_init(&pwm_obj_motor1, PIN_MOTOR_1_PWM, NULL); 		     // Initialize PWM on the supplied pin
	cyhal_pwm_set_duty_cycle(&pwm_obj_motor1, 0.0f, PWM_FREQ_MOTOR1_HZ); // Set a duty cycle and frequency
	cyhal_pwm_register_callback(&pwm_obj_motor1, ISR_PWM_motor1, (void*)&motor1_state);
	cyhal_pwm_enable_event(&pwm_obj_motor1, CYHAL_PWM_IRQ_TERMINAL_COUNT, ISR_PWM_MOTOR_PRIORITY, true); // The event is executed each time the PWM period has passed: page 2-3 https://www.infineon.com/dgdl/Infineon-Component_PWM_V2.20-Software%20Module%20Datasheets-v03_03-EN.pdf?fileId=8ac78c8c7d0d8da4017d0e801c7611bd
	cyhal_pwm_start(&pwm_obj_motor1);

	// MOTOR 1 DISTANCE / SPEED
	cyhal_gpio_callback_data_t gpio_cb_data_motor1_speed =
	{
		.callback     = ISR_GPIO_motor1_speed
	};
	cyhal_gpio_register_callback(PIN_MOTOR_1_SPEED, &gpio_cb_data_motor1_speed);
	cyhal_gpio_enable_event(PIN_MOTOR_1_SPEED, CYHAL_GPIO_IRQ_BOTH, 3, true);

	// ULTRASONIC SENSOR
	// INIT A TIMER FOR THE ULTRASONIC SENSOR ECHO AND THE GPIOS FOR THE TRIG AND ECHO PINS
	cyhal_timer_t timer_obj_ultrasonic;
	const cyhal_timer_cfg_t timer_cfg_ultrasonic =
	{
		.compare_value = 0,                               // Timer compare value, not used
		.period        = TIMER_PERIOD_TICKS_ECHO_TIMEOUT, // Timer period set to a large enough value to expire the timer when no echo is received
		.direction     = CYHAL_TIMER_DIR_UP,              // Timer counts up
		.is_compare    = false,                           // Don't use compare mode
		.is_continuous = false,                           // ONE-SHOT: Do not run timer indefinitely
		.value         = 0                                // Initial value of counter
	};
	cyhal_timer_init(&timer_obj_ultrasonic, NC, NULL);
	cyhal_timer_configure(&timer_obj_ultrasonic, &timer_cfg_ultrasonic); // Apply timer configuration such as period, count direction, run mode, etc.
	cyhal_timer_set_frequency(&timer_obj_ultrasonic, TIMER_FREQ_TICKS);  // Set the frequency of timer to 10000 counts in a second or 10000 Hz
	cyhal_timer_register_callback(&timer_obj_ultrasonic, ISR_TIMER_ultrasonic_echo, (void*)&timer_obj_ultrasonic);
	cyhal_timer_enable_event(&timer_obj_ultrasonic, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);

	Ultrasonic_State ultrasonic_state;
	ISR_GPIO_ultrasonic_echo_arg isr_gpio_ultrasonic_echo_arg = { .ultrasonic_state = &ultrasonic_state, .motor_state = &motor1_state, .timer_obj_echo = &timer_obj_ultrasonic };
	cyhal_gpio_callback_data_t gpio_cb_data_ultrasonic =
	{
		.callback     = ISR_GPIO_ultrasonic_echo,
		.callback_arg = (void*)&isr_gpio_ultrasonic_echo_arg
	};
	cyhal_gpio_init(TRIG_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
	cyhal_gpio_init(ECHO_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);                   // No drive because the sensor module already contains a pull up resistor
	cyhal_gpio_register_callback(ECHO_PIN, &gpio_cb_data_ultrasonic);
	cyhal_gpio_enable_event(ECHO_PIN, CYHAL_GPIO_IRQ_BOTH, ISR_GPIO_ULTRASONIC_ECHO_PRIORITY, true); // Enable events on both edges of the signal

	Ultrasonic_trigger_send(&timer_obj_ultrasonic); // Send the first trigger signal which causes the echo/trigger ISR loop to start

	// OLED SSD1306
	SSD1306_State ssd1306_state = { .drawing_state = NOT_DRAWING };
	cyhal_i2c_t i2c_master_obj;
	cyhal_i2c_cfg_t i2c_master_config = // Define the I2C master configuration structure
	{
		.is_slave = CYHAL_I2C_MODE_MASTER,
		.address = 0, // address is not used for master mode
		.frequencyhal_hz = I2C_MASTER_FREQUENCY
	};

	cyhal_i2c_init(&i2c_master_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL); // Initialize I2C master, set the SDA and SCL pins and assign a new clock
	cyhal_i2c_configure(&i2c_master_obj, &i2c_master_config); // Configure the I2C resource to be master

	// printf("Searching the OLED SSD1306 I²C address => should be: hex 0x3C => dec 60 => bin 011 1100\n\r");
	uint8_t i2c_ack; // Used in the slave address searching loop to check if an I²c device has responded
	uint8_t slave_addr = 0;
	for(; slave_addr <= 0X7F; ++slave_addr)
	{
		// Run the address sweeper to retreive the address of the slave (we only use 1 device)
		// Else you need to check the device/datasheet what the slave address is!

		// send_stop parameter is true because we close the communication immediately after receiving it's address
		if(CY_RSLT_SUCCESS == cyhal_i2c_master_read(&i2c_master_obj, slave_addr, &i2c_ack, 1u, 10u, true)) // OLED SSD1306 doesn't support read mode over I2C! => This still works for retreiving the address
		{
			printf("Slave responded at address: %#04x\n\r", slave_addr);
			break; // Break the search because there is only 1 slave device and that is the oled ssd1306
		}
	}

	// 1.) INIT SSD1306
	// ???? WHY DO I NEED TO SEND THE 'ANNOUNCE COMMAND AGAIN AFTER EACH CALL TO cyhal_i2c_master_write() ????
	// ???? DOES THE cyhal_i2c_master_write() AUTOMATICALLY SEND A STOP CONDITION TO THE SLAVE ON RETURN ????
	// ???? THEN WHAT IS THE STOP CONDITION PARAMETER OF cyhal_i2c_master_write()??? IS THIS TO GENERATE A STOP/START BETWEEN EACH SEND BYTE ????
	SSD1306_Init(&i2c_master_obj, slave_addr, true);

	// 2.) TURN ON THE DISPLAY
	SSD1306_Disp_On(&i2c_master_obj, slave_addr, true, true);

	// 3.) CLEAR THE GDDRAM TO CLEAR DEFAULT NOISE
	SSD1306_Clear_GDDRAM(&i2c_master_obj, slave_addr, gddram_buffer, PIXEL_ON); // Clear the framebuffer and GDDRAM with all '0's to see a nice empty screen (all pixels of) when calling SSD1306_Display_GDDRAM() instead of noise

	// 4.) SET PAGE AND COLUMN ADDRESS TO THE DESIRED DRAWING AREA
	Page_Address page_address = { 0, 7 };
	Column_Address column_address = { 0, 127 };
	// SSD1306_Set_Page_Address(&i2c_master_obj, slave_addr, page_address); // NOT SETTING THE PAGE ADDRESS'S BECAUSE 0, 7 IS THE DEFAULT AFTER INITIALIZATION
	// SSD1306_Set_Column_Address(&i2c_master_obj, slave_addr, page_address, column_address); // NOT SETTING THE COLUMN ADDRESS'S BECAUSE 0, 127 IS THE DEFAULT AFTER INITIALIZATION
	// Draw_Ultrasonic_Sensor_Distance(&i2c_master_obj, slave_addr, gddram_buffer, 0.73f);    // Drawn on page 0, starting in column 0

	// 5.) DRAWING TIMER: frequency: 30hz, period: 33.33ms
	cyhal_timer_t timer_obj_ssd1306;
	const cyhal_timer_cfg_t timer_cfg_ssd1306 =
	{
		.compare_value = 0,                               // Timer compare value, not used
		.period        = 333, 							  // Frequency: 30hz, Period: 33.33ms, Timer frequency: 10KHz, Timer period: 100us => 33.33ms / 100us = 333timer ticks
		.direction     = CYHAL_TIMER_DIR_UP,              // Timer counts up
		.is_compare    = false,                           // Don't use compare mode
		.is_continuous = true,                            // Keep looping the timer
		.value         = 0                                // Initial value of counter
	};
	cyhal_timer_init(&timer_obj_ssd1306, NC, NULL);
	cyhal_timer_configure(&timer_obj_ssd1306, &timer_cfg_ssd1306);   // Apply timer configuration such as period, count direction, run mode, etc.
	cyhal_timer_set_frequency(&timer_obj_ssd1306, TIMER_FREQ_TICKS); // Set the frequency of timer to 10000 counts in a second or 10000 Hz

	ISR_TIMER_OLED_SSD1306_arg isr_timer_oled_ssd1306_arg = { .ssd1306_state = &ssd1306_state };
	cyhal_timer_register_callback(&timer_obj_ssd1306, ISR_TIMER_OLED_SSD1306, &isr_timer_oled_ssd1306_arg);

	cyhal_timer_enable_event(&timer_obj_ssd1306, CYHAL_TIMER_IRQ_TERMINAL_COUNT, 3, true);
	cyhal_timer_start(&timer_obj_ssd1306);

	// PROGRAM MAIN LOOP
	float adc_out_x_pos = 0.0f;
	// float adc_out_y_pos = 0.0f;
	ADC_Motors_Duty_Cycle adc_motors_duty_cycle = { 0 };

	uint32_t previous_cpu_ticks = Cy_SysTick_GetValue();
	uint32_t current_cpu_ticks = previous_cpu_ticks;
    for (;;)
    {
    	current_cpu_ticks = Cy_SysTick_GetValue();
    	printf("Main loop took %lu CPU ticks\n\r", previous_cpu_ticks - current_cpu_ticks);
    	previous_cpu_ticks = current_cpu_ticks;

    	// Read the ADC conversion result for corresponding ADC channel.
    	adc_out_x_pos = ((cyhal_adc_read(&adc_chan_0_obj) + 2048) / 4096.0f) * 100;
    	adc_motors_duty_cycle.pwm_duty_cycle_motor1 = ((cyhal_adc_read(&adc_chan_0_obj) + 2048) / 4096.0f) * 100;
    	if(!motor1_state.is_emergency_break_triggerd)
    		motor1_state.pwm_duty_cycle = adc_out_x_pos;
		// printf("Joystick x position: %0.2f\n\r", adc_out_x_pos);
    	// printf("Joystick x position: %0.2f\n\r", motor1_state.pwm_duty_cycle);

		// adc_out_y_pos = ((cyhal_adc_read(&adc_chan_1_obj) + 2048) / 4096.0f) * 100;
		// printf("Joystick y position: %0.2f\n\r", adc_out_y_pos);

    	if(ssd1306_state.drawing_state)
    	{
    		// DRAW ULTRASONIC DISTANCE
    		// RESET PAGE AND COLUMN ADDRESS TO RECONFIG THE PAGE AND COLUMN POINTER OF THE SSD1306 GGDRAM
    		page_address.start = 0; // TO DO: USE COMMAND MACROS
    		// page_address.end = 7; // TO DO: USE COMMAND MACROS
    		page_address.end = 0; // TO DO: USE COMMAND MACROS
    		column_address.start = 0; // TO DO: USE COMMAND MACROS
    		column_address.end = 127; // TO DO: USE COMMAND MACROS
    		SSD1306_Set_Page_Address(&i2c_master_obj, slave_addr, page_address); // NOT SETTING THE PAGE ADDRESS'S BECAUSE 0, 0 IS THE DEFAULT AFTER INITIALIZATION
    		SSD1306_Set_Column_Address(&i2c_master_obj, slave_addr, page_address, column_address); // NOT SETTING THE COLUMN ADDRESS'S BECAUSE 0, 127 IS THE DEFAULT AFTER INITIALIZATION
    		Draw_Ultrasonic_Sensor_Distance(&i2c_master_obj, slave_addr, gddram_buffer, ultrasonic_state.distance);    // Drawn on page 0, starting in column 0

    		// DRAW MOTOR1 DUTY CYCLE
    		// MOVE TO THE NEXT PAGE AND RESET THE COLUMN ADDRESS TO RECONFIG THE PAGE AND COLUMN POINTER OF THE SSD1306 GGDRAM
			page_address.start = 1; // TO DO: USE COMMAND MACROS
			// page_address.end = 7; // TO DO: USE COMMAND MACROS
			page_address.end = 1; // TO DO: USE COMMAND MACROS
			column_address.start = 0; // TO DO: USE COMMAND MACROS
			column_address.end = 127; // TO DO: USE COMMAND MACROS
			SSD1306_Set_Page_Address(&i2c_master_obj, slave_addr, page_address); // NOT SETTING THE PAGE ADDRESS'S BECAUSE 0, 0 IS THE DEFAULT AFTER INITIALIZATION
			SSD1306_Set_Column_Address(&i2c_master_obj, slave_addr, page_address, column_address); // NOT SETTING THE COLUMN ADDRESS'S BECAUSE 0, 127 IS THE DEFAULT AFTER INITIALIZATION
			Draw_Motor_Duty_Cycle(&i2c_master_obj, slave_addr, gddram_buffer, motor1_state.pwm_duty_cycle);    // Drawn on page 1, starting in column 0

    		ssd1306_state.drawing_state = NOT_DRAWING;
    	}
    }
}

void TIMER_ultrasonic_echo_start(cyhal_timer_t* timer_obj)
{
	cyhal_timer_start(timer_obj);
}

uint32_t TIMER_ultrasonic_echo_read(cyhal_timer_t* timer_obj)
{
	return cyhal_timer_read(timer_obj);
}

void Ultrasonic_trigger_send(cyhal_timer_t* timer_obj)
{
	cyhal_gpio_write(TRIG_PIN, 1U);
	cyhal_system_delay_us(10);
	cyhal_gpio_write(TRIG_PIN, 0U);
	TIMER_ultrasonic_echo_start(timer_obj);
}

void ISR_GPIO_joystick(void* handler_arg, cyhal_gpio_event_t event)
{
    CY_UNUSED_PARAMETER(event);

    cyhal_timer_t* timer_obj_joystick = (cyhal_timer_t*)handler_arg;
    cyhal_timer_reset(timer_obj_joystick); // Reset the timer to 0
    cyhal_timer_start(timer_obj_joystick); // Restart the timer

    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
}

void ISR_TIMER_joystick(void* callback_arg, cyhal_timer_event_t event)
{
    (void)callback_arg;
    (void)event;

    cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
}

// IMPORTANT:
// ONLY THE ULTRASONIC SENSOR CAN INTERRUPT THIS ISR TO PERFORM AN EMERGENCY BREAK IF AN OBJECT IS TO CLOSE!
// IF SOMETHING NONE IMPORTANT INTERRUPTS THIS ISR BEFORE IT GOT THE CHANCE TO MEASURE THE ACTUAL TIME, THE TIMER WOULD KEEP RUNNING AND SO BY THE TIME THIS ISR IS ALLOWED TO RUN AGAIN WERE IT LEFT OF, THE TIME MEASURED IS LONGER THEN IT WAS WHEN THIS ISR SHOULD HAVE READ THE TIME AND SO THE WHEEL SEEMED TO SPIN SLOWER WHILE IT WASN'T ACTUALLY SPINNGING SLOWER !!!!
// VERY DANGEROUS WHEN COUPLED TO A CRUISE CONTROL FOR EXAMPLE, BECAUSE THE CRUISE CONTROL COULD SPEED UP THE MOTOR WHILE THE MOTOR WAS ALREADY SPINNING AT THE RIGHT SPEED !
void ISR_GPIO_motor1_speed(void* handler_arg, cyhal_gpio_event_t event)
{
	// THE SPEED DISK OF THE SENSOR HAS 20 HOLES. IR LIGHT WILL PASS THROUGH THE HOLES AND GET BLOCKED BY THE SPOKES.
	// EACH STATE CHANGE OF THE IR RECEIVER WILL TRIGGER AN INTERRUPT BECAUSE THIS ISR IS SCHEDULED FOR BOTH FALLING AND RISING EDGE.
	// IF ALL INTERRUPTS ARE COUNTED, 40 INTERRUPTS SHOULD HAVE HAPPEND FOR 1 WHEEL REVOLUTION.
	static uint8_t interrupt_count = 0;
	static double distance_traveled_m = 0;
	static time_t previous_revolution_time = 0;
	static time_t current_revolution_time = 0;

	++interrupt_count;

	if(interrupt_count == 40) // FULL WHEEL REVOLUTION COMPLETED => only adding the distance and calculating the speed each wheel revolution to not overload the CPU because this interrupt is triggered at a pretty fast rate.
	{
		// Calculate the distance traveled
		distance_traveled_m += ((2 * PI * WHEEL_RADIUS) / 100.0f); // Distance += wheel circumference
		interrupt_count = 0;
		// printf("Distance traveled: %.2fm\n\r", distance_traveled_m);

		// Calculate the speed
	}
}

void ISR_GPIO_Line_Detect(void* handler_arg, cyhal_gpio_event_t event)
{
	(void)handler_arg;
	(void)event;

	if(event == CYHAL_GPIO_IRQ_RISE)        // Detect absorbing surface (Darker colors)
		// // cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
		; // printf("Line detect rise event: absorbing surface\n\r");
	else if (event == CYHAL_GPIO_IRQ_FALL)  // Detect reflecting surface
		// // cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
		; // printf("Line detect fall event: reflecting surface\n\r");
}

void ISR_PWM_motor1(void* callback_arg, cyhal_pwm_event_t event)
{
	Motor_State* motor1_state = (Motor_State*)callback_arg;
	if(motor1_state->is_emergency_break_triggerd == false)
		cyhal_pwm_set_duty_cycle(motor1_state->pwm_obj, motor1_state->pwm_duty_cycle, PWM_FREQ_MOTOR1_HZ);
}

void ISR_GPIO_ultrasonic_echo(void* callback_arg, cyhal_gpio_event_t event)
{
	ISR_GPIO_ultrasonic_echo_arg* args = (ISR_GPIO_ultrasonic_echo_arg*)callback_arg;
	static uint32_t ultrasonic_echo_time = 0;

	if(event == CYHAL_GPIO_IRQ_RISE) // Save the time when the echo pin goes high to wait for the received echo
	{
		ultrasonic_echo_time = TIMER_ultrasonic_echo_read(args->timer_obj_echo);
	}
	else if (event == CYHAL_GPIO_IRQ_FALL) // Get the current time and calculate how long it took for the sonic wave to make its way back to the sensor
	{
		// float ultrasonic_distance = (TIMER_ultrasonic_echo_read(args->timer_obj_echo) - ultrasonic_echo_time) / 58.0f; // or * 0.0343 / 2
		float* ultrasonic_distance = &(args->ultrasonic_state->distance);
		*ultrasonic_distance = (TIMER_ultrasonic_echo_read(args->timer_obj_echo) - ultrasonic_echo_time) / 58.0f; // or * 0.0343 / 2
		// printf("Distance: %.2fm\n\r", *ultrasonic_distance);

		if(*ultrasonic_distance < 0.1f) // IF OBJECT IS TOO CLOSE, THE MOTOR NEEDS TO STOP SPINNING IMMEDIATELY => LATER FEATURE: MOTOR CAN SPIN BACKWARDS TO MOVE FROM THE OBJECT BUT CAN'T MOVE FORWARDS
		{
			args->motor_state->is_emergency_break_triggerd = true;
			args->motor_state->pwm_duty_cycle = 0.0f;
			cyhal_pwm_set_duty_cycle(args->motor_state->pwm_obj, 0.0f, PWM_FREQ_MOTOR1_HZ);
		}
		else
		{
			args->motor_state->is_emergency_break_triggerd = false;
		}

		ultrasonic_echo_time = 0;                      // Reset the echo time
		Ultrasonic_trigger_send(args->timer_obj_echo); // Send trigger again
	}
}

void ISR_TIMER_ultrasonic_echo(void* callback_arg, cyhal_timer_event_t event)
{
	cyhal_timer_t* timer_obj = (cyhal_timer_t*)callback_arg;

	if(event == CYHAL_TIMER_IRQ_TERMINAL_COUNT) // Timer time-out reached => no echo is received
		Ultrasonic_trigger_send(timer_obj);     // Send trigger again
}

void ISR_TIMER_OLED_SSD1306(void* callback_arg, cyhal_timer_event_t event)
{
	ISR_TIMER_OLED_SSD1306_arg* args = (ISR_TIMER_OLED_SSD1306_arg*)callback_arg;

	args->ssd1306_state->drawing_state = IS_DRAWING;// SET A FLAG FOR THE MAIN LOOP TO DRAW ON THE OLED
}
