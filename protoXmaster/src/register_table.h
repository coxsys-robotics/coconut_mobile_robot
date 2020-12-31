#ifndef REGISTER_TABLE_H
#define REGISTER_TABLE_H
//////// PROTOX: CONSTRAIN: in the beginning 4 bytes of each struct(section) should to have uint32_r seq = 0;

struct ULTRASONIC				// sizeof(ULTRASONIC) = 20 Bytes
{
		/**	ULTRASONIC
		 * 			Front Robot
		 * 			  	^
		 * 			  	|
		 * 	 _______________________
		 * 	/	1	2  		3	4	\
		 * 	|						|
		 * 	|						|
		 * 	|						|
		 * 	|						|
		 * 	|			O			|
		 * 	|						|
		 * 	|						|
		 * 	|						|
		 * 	\10					  5/
		 * 	 \--9----8----7----6--/
		 *
		 *			Top view
		 * */
		uint16_t dist1;				// unit : centimeter
		uint16_t dist2;				// unit : centimeter
		uint16_t dist3;				// unit : centimeter
		uint16_t dist4;				// unit : centimeter
		uint16_t dist5;				// unit : centimeter
		uint16_t dist6;				// unit : centimeter
		uint16_t dist7;				// unit : centimeter
		uint16_t dist8;				// unit : centimeter
		uint16_t dist9;				// unit : centimeter
		uint16_t dist10;			// unit : centimeter
};

struct VEL_ROBOT					// sizeof(VEL_ROBOT) = 8 bytes
{
		float linear_vel_x;			// unit : m/s
		float angular_vel_theta;	// unit : rad/s
};

struct CMD_NEOPIXEL					// sizeof(CMD_NEOPIXEL) = 24 bytes
{
		uint16_t ms_time[3];	//	unit: millsecond	// at least 100 ms
		uint16_t num[3];		//	number of pixel
		/**	mode of Neopixel display
		 * 		0				|	All pixel use rgb[0]
		 * 		1				|	All pixel blink between 2 color ==> on rgb[0] with ms_time[0] => on rgb[1] with ms_time[1] and loop
		 * 		2				|	"---------------------" 3 color ==> on rgb[0] with ms_time[0] => on rgb[1] with ms_time[1]
		 * 						|	=> on rgb[2] with ms_time[2] and loop
		 *	-------------------------------------------------------------------------------------------
		 * 		3				|	Pattern 2 color ==> on rgb[0] with num[0] pixel => on rgb[1] with num[1] pixel and loop
		 * 		4				|	Pattern 3 color ==> on rgb[0] with num[0] pixel => on rgb[1] with num[1] pixel => on rgb[2] with num[2] and loop
		 *	#### Example Pattern 3 color ####
		 *		determine num[0] = 50, num[1] = 40, num[2] = 10
		 *	pixel number > 1 ... rgb[0].. 50 > 51 ... rgb[1] ... 90 > 91 ... rgb[2] ... 100 > 101 ... rgb[0] ... 150
		 *	> 151 ... rgb[1] ... >> and loop
		 *	-------------------------------------------------------------------------------------------
		 * 		5				|	Running Light 1 tab with color rgb[0], size of tab = num[0], by background light is rgb[1]
		 * 		6				|	Running Light 2 tab.1st with color rgb[0] (size of tab = num[0])
		 * 						|		+ 2nd opposite position and the "same direction" tab with color rgb[1] (size of tab = num[1])
		 * 						|		, by background light is rgb[2]
		 * 		7				|	Like mode 6 but 2nd tab is "reverse direction" of 1st tab
		 * 		Running Light flow {CMD_NEOPIXEL.step} pixel per every ms_time[0]
		 * --------------------------------------------------------------------------------------------
		 * 		8				|	Fade down
		 * 		9				|	Smooth change 2 color, rgb[0] -> rgb[1] ====> rgb[0] and loop
		 * 		10				|	Smooth change 3 color, rgb[0] -> rgb[1] -> rgb[2] ====> rgb[0] and loop
		 * 		253				|	Charging status
		 * 		254				|	Battery low (Warning)
		 * */
		uint8_t mode;			//	mode Neopixel.
		uint8_t r[3];			//	0 - 255
		uint8_t g[3];			//	0 - 255
		uint8_t b[3];			//	0 - 255
		uint16_t step;			//	step of mode Light flow.
};

struct Robot_Info
{
		uint32_t seq = 0;				// sequence of read to (Robot_Info)infomation section.
		uint16_t firmware_version;		// Robot firmware version.
		uint16_t control_time_period;	// period control MotorDriver.
};
// sizeof(emu_EERPOM) = 8 bytes

struct Status
{
		uint32_t seq = 0;				// sequence of read/write to status section.NOTE: seq don't count the auto return.
		uint16_t update_period = 1000;	// use for set auto return. // unit : Hz
		/**	Device_status
		 * 		0b00000001		|	Push freewheel button
		 * 		0b00000010		|	-
		 * 		0b00000100		|	-
		 * 		0b00001000		|	-
		 * 		0b00010000		|	-
		 * 		0b00100000		|	-
		 * 		0b01000000		|	-
		 * 		0b10000000		|	-
		 * */
		uint8_t device_status;
		/** powerboard_status
		 * 		0				|	Off
		 * 		1				|	IDLE (ON)
		 * 		2				|	Charging
		 * 		3				|	Init
		 * 		4				|	Shutdown
		 * */
		uint8_t powerboard_status;
		/**	battery_status
		 * 		0				|	Normal
		 * 		1				|	Charging
		 * 		2				|	Charged
		 * 		3				|	Error
		 * */
		uint8_t battery_status;
		/** Actuator_status [All device use 24V+ (Motor + UVC)]
		 * 		0				|	OFF
		 * 		1				|	ON
		 * 		2				|	INIT
		 * 		3				|	push Emergency button
		 * 		4				|	Application command OFF
		 * 		5				|	Application command ON
		 * 		50				|	MotorDrive OVER-CURRENT
		 * 		51				|	MotorDrive OVER-VOLTAGE
		 * 		52				|	MotorDrive Encoder Failure
		 * 		53				|	MotorDrive OVER HEAT
		 * 		54				|	MotorDrive UNDER-VOLTAGE
		 * 		55				|	MotorDrive OVER LOAD
		 * 		100				|	Reset Alarm
		 * */
		uint8_t actuator_status;
		/**	If no Error =====> PowerBoard_ErrorCode = 0b00000000
		 * 		0b00000001		|	Thermal error
		 * 		0b00000010		|	Voltage error
		 * 		0b00000100		|	CAN receive message error
		 * 		0b00001000		|	CAN send message error
		 * 		0b00010000		|	Emergency button is pushed
		 * 		0b00100000		|	-
		 * 		0b01000000		|	-
		 * 		0b10000000		|	Unexpected error
		 * */
		uint8_t powerboard_errorcode;
		/** If no Error =====> MCU_ErrorCode = 0b00000000
		 * 		0b00000001 		|	PowerBoard don't send heartbeat
		 * 		0b00000010		|	Wheel Left don't send heartbeat
		 * 		0b00000100		|	Wheel Right don't send heartbeat
		 * 		0b00001000		|	CAN initial error
		 * 		0b00010000		|	ADC initial error
		 * 		0b00100000		|	SPI initial error
		 * 		0b01000000		|	--
		 * 		0b10000000		|	--
		 * */
		uint8_t mcu_errorcode;

		uint8_t battery_percent;		// unit : percent 0 - 100 %
		uint8_t reserve1;
		uint8_t reserve2;
		uint8_t reserve3;

		float sens_voltage;				// unit : Volt
		float sens_current;				// unit : Amp
		float sens_celsius;				// unit : Degree Celsius
};
// sizeof(Status) = 28 bytes

struct Sensor
{
		uint32_t seq = 0;		// sequence of read/write to sensor section.NOTE: seq don't count the auto return.

		uint16_t update_period = 25;	// use for set auto return. // unit : Hz
		/**	Bumper_status
		 * 		0b00000001		|	Push bumper Front Left
		 * 		0b00000010		|	Push bumper Front Middle
		 * 		0b00000100		|	Push bumper Front Right
		 * 		0b00001000		|	Push bumper Back Right
		 * 		0b00010000		|	Push bumper Back Middle
		 * 		0b00100000		|	Push bumper Back Left
		 * */
		uint8_t bumper_status;
		uint8_t reserve1;

		VEL_ROBOT feedback_vel;			// 8 bytes
		ULTRASONIC ult;					// 20 bytes
};
// sizeof(Sensor) = 36 bytes

struct COMMAND
{
		uint32_t seq = 0;		// sequence of write to command section.
		VEL_ROBOT cmd_robot;			// 8 bytes
		/**	light_on
		 * 		0b00000001 		|	on Front light
		 * 		0b00000010		|	on Turn Left light
		 * 		0b00000100		|	on Turn Right light
		 * 		0b00001000		|	on Rear light
		 * 		0b00010000		|	-
		 * 		0b00100000		|	-
		 * 		0b01000000		|	-
		 * 		0b10000000		|	-
		 * */
		uint8_t light_on;
		uint8_t buzzer_melody;
		uint8_t reserve1;
		uint8_t reserve2;
		CMD_NEOPIXEL neopixel;			// 24 bytes
};
// sizeof(COMMAND) = 40 bytes

struct REGISTER_TABLE
{
		Robot_Info info;			// section1	// R
		Status status;				// section2	// RW
		Sensor sens;				// section3	// RW
		COMMAND command;			// section5	// W
};
// sizeof(REGISTER_TABLE) = 112 bytes
#endif // REGISTER_TABLE_H

/**	buzzer_melody
 * 			0				|	silence
 * 			1				|	G3
 * 			2				|	GS3
 * 			3				|	A3
 * 			4				|	AS3
 * 			5				|	B3
 * 			6				|	C4	///////////////////////////
 * 			7				|	CS4
 * 			8				|	D4
 * 			9				|	DS4
 * 			10				|	E4
 * 			11				|	F4
 * 			12				|	FS4
 * 			13				|	G4
 * 			14				|	GS4
 * 			15				|	A4
 * 			16				|	AS4
 * 			17				|	B4
 * 			18				|	C5	////////////////////////////
 * 			19				|	CS5
 * 			20				|	D5
 * 			21				|	DS5
 * 			22				|	E5
 * 			23				|	F5
 * 			24				|	FS5
 * 			25				|	G5
 * 			26				|	GS5
 * 			27				|	A5
 * 			28				|	AS5
 * 			29				|	B5
 * 			30				|	C6	////////////////////////////
 * 			31				|	CS6
 * 			32				|	D6
 * 			33				|	DS6
 * 			34				|	E6
 * 			35				|	F6
 * 			36				|	FS6
 * 			37				|	G6
 * 			38				|	GS6
 * 			39				|	A6
 * 			40				|	AS6
 * 			41				|	B6
 * 			42				|	C7	////////////////////////////
 * 			43				|	CS7
 * 			44				|	D7
 * 			45				|	DS7
 * 			46				|	E7
 * 			47				|	F7
 * 			48				|	FS7
 * 			49				|	G7
 * 			50				|	GS7
 * 			51				|	A7
 * 			52				|	AS7
 * 			53				|	B7
 * 			54				|	C8	////////////////////////////
 * 			55				|	CS8
 * 			56				|	D8
 * 			57				|	DS8
 * 			58				|	E8
 * 			59				|	F8
 * 			60				|	FS8
 * 			61				|	G8
 * 			62				|	GS8
 * 			63				|	A8
 * 			64				|	AS8
 * 			65				|	B8
 * */