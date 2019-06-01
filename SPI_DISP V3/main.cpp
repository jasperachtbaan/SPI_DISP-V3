/*
 * SPI_DISP V3.cpp
 *
 * Created: 7/30/2017 6:28:04 PM
 * Author : jaspe
 */ 
#define F_CPU 32000000
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#define MOSI_PIN 5
#define SS_PIN 4
#define SCK_PIN 7
#define RS_PIN 3
#define TCC1_MAX 31250
//Some dividers, can be changed if you have to rotate the knob too fast/slow.
#define BRIGHTNESS_DIV 4
#define CHANNEL_DIV 1
#define MODE_DIV 1
//How long it takes before the dimmer goes into edit mode
#define BUTTON_DELAY_MS 250.0
#define INVERT_BUTTON


#define BUTTON_DELAY (uint16_t)(BUTTON_DELAY_MS * 31.25);
enum DMXMANMode { DMX, MAN };
enum encoderMode { OFF, MODESEL, CHANSEL, BRIGHTSEL };
DMXMANMode currentMode;
encoderMode EncoderMode;

volatile uint8_t currentSelector = 0;
volatile bool editMode = false;
bool DMXBlinkVal0 [33] = {0,0,0,0,0,0,1,1,1};
bool DMXBlinkVal1 [33] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1};
bool DMXBlinkVal2 [33] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1};
bool DMXBlinkVal3 [33] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1};
bool MANBlinkVal0 [33] = {0,0,0,0,0,0,1,1,1,1,1,1};
bool MANBlinkVal1 [33] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1};
bool NULLBlinkVal [33];
//All master display arrays
char currDispData [33];
bool* currBlinkMaskPtr;
uint8_t currCursorPos = 0;
//To be display arrays, the arrays that will be set to the display in updateDisp()
char DMXdisp [33] = "Mode: DMX       Chan:     &     ";
char MANdisp [33] = "Mode: Manual    Brightness:    %";
char* toBeDispPtr;
char oldToBeDispData [33]; //Array for change detection in display data
bool lt = false;
volatile uint16_t editCounter = 0; 
volatile uint16_t DMXErrCnt = 0; 
volatile bool DMXErrFlag = false;
volatile uint16_t brightness = 0;
volatile int16_t DMXChan = 1;
volatile int cnt = 0;
volatile uint16_t finalRes = 0; 
volatile uint8_t dmxMult = 1;
#define DMXMax 511
#define DMXmin 1
volatile bool CCA_flag = false;
volatile bool CCB_flag = false;
volatile bool CCA_firstSam = false;
volatile bool CCB_firstSam = false;
volatile bool OVF_firstSam = false;
volatile bool blinkTimeoutIgnore = false;
//Bool when the button has to wait for a long press
volatile bool longPressDetection = false;




//EEPROM addresses
#define DMX_addr (uint16_t*)0x15
#define MODE_addr (uint8_t*)0x10
#define BRIGHTNESS_addr (uint16_t*)0x13


// EEPROM addresses
// #define DMX_addr 0x1215
// #define MODE_addr 0x1212
// #define BRIGHTNESS_addr 0x1213

//All setup functions
void clk_set_32MHz(){
	OSC_CTRL |= OSC_RC32MEN_bm; //Setup 32Mhz crystal
	while(!(OSC_STATUS & OSC_RC32MRDY_bm));
	_delay_ms(1);
	CCP = CCP_IOREG_gc; //Trigger protection mechanism
	CLK_CTRL = CLK_SCLKSEL_RC32M_gc; //Enable internal  32Mhz crystal (1)
	_delay_ms(1);
}
//Except for send_SPI
inline void send_SPI(uint8_t SPIdata){
	PORTC_OUTCLR = (1 << SS_PIN);
	SPIC_DATA = SPIdata;
	while(!(SPIC_STATUS & SPI_IF_bm)){}
	SPIC_STATUS |= SPI_IF_bm;
	PORTC_OUTSET = (1 << SS_PIN);
	_delay_us(25);
}



void setup_blink(){
	OSC_CTRL |= OSC_RC32KEN_bm;
	do {
		/* Wait for the 32kHz oscillator to stabilize. */
	} while ( ( OSC_STATUS & OSC_RC32KRDY_bm ) == 0);
	CLK_RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm; 
	while(RTC_STATUS & RTC_SYNCBUSY_bm){}
	RTC_CTRL = RTC_PRESCALER_DIV1_gc;
	while(RTC_STATUS & RTC_SYNCBUSY_bm){}
	RTC_INTCTRL = RTC_OVFINTLVL_LO_gc;
	RTC_PER = 1023;
	while(RTC_STATUS & RTC_SYNCBUSY_bm){}
}

void setup_SPI(){
	SPIC_CTRL = SPI_MODE_3_gc | SPI_PRESCALER_DIV64_gc | SPI_ENABLE_bm | SPI_MASTER_bm;
	PORTC_OUTSET = (1 << SS_PIN);
}

void setup_disp(){
	PORTC_DIRSET = (1 << MOSI_PIN) | (1 << SS_PIN) | (1 << SCK_PIN) | (1 << RS_PIN); //Set all pins used for communication to output
	send_SPI(0b1100); //Turn display on
	send_SPI(0b111000); //Set display to two line mode
	send_SPI(0b1); //Clear display and reset cursor
	_delay_us(1300);
}

void setup_btn(){
	//Setup TCD0 to overflow when PC2 has been low for 1 second
	TCD0_CTRLA = TC_CLKSEL_DIV1024_gc;
	TCD0_CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH3_gc;
	TCD0_PER = BUTTON_DELAY;//TCD0 will overflow in exactly ... seconds
	TCD0_INTCTRLA = TC_OVFINTLVL_LO_gc;
	TCD0_CCA = 1;
	TCD0_CCB = 0;
	//Setup PC2 to HIGH level event trigger and pull up ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
	PORTC_OUTCLR = (1 << 2);
	PORTC_PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_LEVEL_gc;
	#ifdef INVERT_BUTTON
		PORTC_PIN2CTRL |= PORT_INVEN_bm;
	#endif

	//Setup event channel 3 for PC2
	EVSYS_CH3MUX = EVSYS_CHMUX_PORTC_PIN2_gc;
	//Hold the counter 0 for as long as the button is not pressed. When the counter reaches a compare value an interrupt will be triggered. The INVEN bit is immediately set so the event system
	//will keep the counter at 0 so the interrupt won't be triggered when the button is kept pressed. Another compare value is set so when the button is released again the INVEN bit can be reset
	//and the button is ready for another press.
	//When the DMX controller waits for a long pulse, the INVEN bit is not set yet when the first compare triggers. THe compare is set to a large value (1.5S) and when the compare is reached again the 'long press' code is executed.
	//When the button is released within this period the compare values are reset again by the B compare channel.
}

void setup_int(){
	PMIC_CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
}

void setup_PWM(){
	//Setup PWM for 14 bits to enable faster PWM
	TCE0_CCA = 0;
	TCE0_CTRLA = 0b1; //Set CLK
	TCE0_CTRLB = 0b11 | (1 << 4); //Set singleslope PWM and enable PWM on 0C1A
	TCE0_PER = 0x3FFE; //Fixes problem that brightness doesn't go up to 100%
	//Wait for timer to stabilize
	_delay_ms(10);
	PORTE_DIRSET |= (1 << 0);
}

void QDEC_INIT(){
	//Setup
	PORTD_OUTCLR = (1 << 4) | (1 << 5);
	PORTD_PIN4CTRL = PORT_ISC_LEVEL_gc; //Set PD4 to low level sense
	PORTD_PIN5CTRL = PORT_ISC_LEVEL_gc; //Set PD5 to low level sense

	//Setup event system
	EVSYS_CH0MUX = EVSYS_CHMUX_PORTD_PIN4_gc; //Setup EVCH0 to PD4
	EVSYS_CH1MUX = EVSYS_CHMUX_PORTD_PIN5_gc; //Setup EVCH1 to PD4
	EVSYS_CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;
	EVSYS_CH1CTRL = 0;

	//Setup TCC0 for Qdec decoding
	TCC0_CTRLA = TC_CLKSEL_DIV1_gc;
	TCC0_CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc; //Ev channel 0 for QDEC and enable QDEC
	TCC0_CCA = 1;
	#define TC0_CCB_CHANNEL_MODE (int)((320 * CHANNEL_DIV) - 1)
	TCC0_CCB = TC0_CCB_CHANNEL_MODE;
	//TCC0_PER = 770;
}



void DMX_init(){
	//Setup event system to reset TCC1 on any edge of PD2
	PORTD_DIRCLR = (1 << 2);
	PORTD_PIN2CTRL = PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc | PORT_INVEN_bm; //Setup pin for DMX dead time timing
	EVSYS_CH2MUX = EVSYS_CHMUX_PORTD_PIN2_gc; 
	//Enable TCC1
	TCC1_CTRLA = TC_CLKSEL_DIV8_gc; //Set CLK/8
	TCC1_CTRLB = TC_WGMODE_NORMAL_gc;//Set WGM to FRQ
	//TCC1_INTCTRLA = TC_OVFINTLVL_HI_gc; //Set OVF int priority to HIGH
	TCC1_CTRLD = TC_EVACT_RESTART_gc | TC_EVSEL_CH2_gc;
	TCC1_PER = 349; //Interrupt every 87 us
	//Setup USART
	//USARTD0_CTRLA = USART_RXCINTLVL_MED_gc;
	//USARTD0_CTRLB = USART_RXEN_bm;
	USARTD0_CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_SBMODE_bm; //Two stop bits
	USARTD0_BAUDCTRLA = 0b10000000;//Set baud to 250000
	USARTD0_BAUDCTRLB = 0b10010011;//Set baud to 250000
}

//All other functions

void update_EEPROM_RAM(){ //Updates EEPROM with all brightness, mode and DMX information from values stored in the RAM
	uint8_t modeByte = 0;	
	if(currentMode == MAN){
		modeByte = 0xFF;
	}
	if(eeprom_read_word(BRIGHTNESS_addr) != brightness){
		eeprom_write_word(BRIGHTNESS_addr, brightness);
	}
	if(eeprom_read_word(DMX_addr) != DMXChan){
		eeprom_write_word(DMX_addr, DMXChan);
	}
	if(eeprom_read_byte(MODE_addr) != modeByte){
		eeprom_write_byte(MODE_addr, modeByte);
	}
}

inline void DMX_DISABLE(){//Disable all DMX interrupts
	TCC1_INTCTRLA &= ~TC_OVFINTLVL_HI_gc;//Disable timer0 interrupt
	USARTD0_CTRLA &= ~USART_RXCINTLVL_MED_gc;//Disable USART interrupt
	USARTD0_CTRLB &= ~USART_RXEN_bm;
	//PORTD_INTCTRL &= ~PORT_INT0LVL_HI_gc;
}

inline void DMX_ENABLE(){//Enable all DMX interrupts
	TCC1_INTCTRLA |= TC_OVFINTLVL_HI_gc;//Enable timer0 interrupt
	USARTD0_CTRLA |= USART_RXCINTLVL_MED_gc;//Enable USART interrupt
	USARTD0_CTRLB |= USART_RXEN_bm;
	//PORTD_INTCTRL |= PORT_INT0LVL_HI_gc;
}

int64_t mathPow(int16_t gnd, int16_t exp){
	int16_t i = 0;
	int64_t result = gnd;
	for(i = 1; i < exp; i++){
		result *= gnd;
	}
	if(exp == 0){
		return 1;
	}
	else{
		return result;
	}
}

inline void WR_CHAR(){
	PORTC_OUTSET = (1 << RS_PIN);
	_delay_us(1); //Delay 1 us to ensure that the LCD has time to process change in RS signal
}

inline void NWR_CHAR(){
	PORTC_OUTCLR = (1 << RS_PIN);
	_delay_us(1); //Delay 1 us to ensure that the LCD has time to process change in RS signal
}

void updateDisp(){
	uint8_t lowBound = 0xFF; //Lowest number in what the two arrays differ
	uint8_t highBound = 0; //Highest number in what the arrays differ
	char temp [33];
	uint32_t i = 0;
	bool toBlink = false;
	for(i = 0; i < 32; i++){//Check if any changes in diplayData have occurred, if so reset blink timer
		if(oldToBeDispData[i] != *(toBeDispPtr + i)){
			RTC_CNT = 0;
			editCounter = 0;
		}
	}
	if(RTC_CNT > 512){
		toBlink = true; //If more than half a second has passed, blink
	}
	for(i = 0; i < 32; i++){//Blink
		if(toBlink && *(currBlinkMaskPtr + i) && editMode){
			temp[i] = ' ';
		}
		else
		{
			temp[i] = *(toBeDispPtr + i);
		}
		oldToBeDispData[i] = *(toBeDispPtr + i);
	}
	for(i = 0; i < 32; i++){
		if(currDispData[i] != temp[i]){//Set the low and high bounds
			currDispData[i] = temp[i];
			highBound = i;
			if(lowBound == 0xFF){
				lowBound = i;
			}
		}
	}
	if(lowBound != 0xFF){ //If anything has changed on the display
		uint8_t tempPos = lowBound;
		if(tempPos > 15){
			tempPos += 48;
		}
		tempPos |= 0b10000000;//Set the necessary bit to set DDRAM address
		send_SPI(tempPos);
		WR_CHAR();
		for(i = lowBound; i <= highBound; i++){
			send_SPI(currDispData[i]);
			if(i == 15 && highBound != 15){
				NWR_CHAR();
				send_SPI(0b11000000); //Set display address to 16
				WR_CHAR();
			}
		}
		NWR_CHAR();
	}
}

inline void setCharInToBe(char charToSend){//Doesn't include RS signal, just to update display arrays correctly
	*(toBeDispPtr + currCursorPos) = charToSend;
	currCursorPos++;
}

void LCD_PRINTDEC(uint16_t numToPrint, uint8_t pos, uint8_t noOfDigits){
	//currCursorPos = pos;
	//WR_CHAR();
	int16_t decCounter = 0;
	int64_t startNum = numToPrint;
	if(noOfDigits == 0xFF){/*0xFF for autodetect digits*/
		bool check = false; //If the correct numbersize has been found
		while(!check){
			if(mathPow(10, decCounter) > numToPrint){
				check = true;
			}
			decCounter++;
		}
		decCounter -= 2; //Subtract one for the extra decCounter++ and one for the fact that if one digit has to be printed decCounter has to be 0
	}
	else{
		decCounter = noOfDigits - 1;//If one digit has to be printed decCounter has to be 0
	}

	int16_t i;
	for(i = decCounter; i >= 0; i--){ //Print every digit
		int16_t t;
		for(t = 0; startNum - (mathPow(10, i) * t) >= 0; t++){} //t = 9 if i = 1 and startnum = 90, t = 3 if i = 2 and startnum is 356
		if(t > 0){
			startNum -= mathPow(10, i) * (t - 1); //Subtract the printed number from the start number
			*(toBeDispPtr + pos + decCounter - i) = 0x2F + t;
		}
		else{
			*(toBeDispPtr + pos + decCounter - i) = 0x30;
		}
		
		
	}
	//NWR_CHAR();
}

void LCD_PRINT(char charBuf[32], uint8_t pos){
	currCursorPos = pos;
	int i = 0;
	bool NULL_FOUND = false;
	for(i = 0; !NULL_FOUND; i++){
		if(charBuf[i] == '\0'){
			NULL_FOUND = true; //Stop printing if NULL byte has been found
		}
		else{
			setCharInToBe(charBuf[i]);
		}
	}
}

void setScrnAndPWM(uint16_t tempVal){ //Update screen with brightness value and update PWM
	uint16_t fullTemp = (double)tempVal * (double)tempVal * 0.11111;
	TCE0_CCABUF = fullTemp >> 2; //14 bits
	uint16_t dispVal = (double)tempVal / 7.68;
	LCD_PRINTDEC(dispVal, 28, 0x3);
}

void updateBrightness(){
	uint16_t tempStore = TCC0_CNT;
	//Set a maximum brightness threshold
	#define MAX_CNT_BRIGHTNESS ((BRIGHTNESS_DIV*3072) + 0.99)
	if(tempStore > MAX_CNT_BRIGHTNESS - 3){
		tempStore = (int)MAX_CNT_BRIGHTNESS;
	}
	//Set a minimum brightness threshold
	#define MIN_CNT_BRIGHTNESS (BRIGHTNESS_DIV*8)
	if(tempStore < MIN_CNT_BRIGHTNESS){
		tempStore = 0;
	}
	#define BRIGHTNESS_FINAL_DIV (BRIGHTNESS_DIV * 4)
	brightness = tempStore / BRIGHTNESS_FINAL_DIV;
	setScrnAndPWM(brightness);
}

//Mode control functions
void setMode(DMXMANMode tempMode){
	if(tempMode == DMX){
		if(editMode){
			currBlinkMaskPtr = DMXBlinkVal0;
		}
		toBeDispPtr = DMXdisp;
		currentMode = DMX;
		DMX_ENABLE();
		LCD_PRINTDEC(DMXChan, 22, 3);
		LCD_PRINTDEC(DMXChan + 1, 28, 3);
	}
	if(tempMode == MAN){
		if(editMode){
			currBlinkMaskPtr = MANBlinkVal0;
		}
		toBeDispPtr = MANdisp;
		currentMode = MAN;
		DMX_DISABLE();
		setScrnAndPWM(brightness);
	}
}

void update_RAM_EEPROM(){ //Updates RAM with all brightness, mode and DMX information from values stored in the EEPROM
	brightness = eeprom_read_word(BRIGHTNESS_addr);
	DMXChan = eeprom_read_word(DMX_addr);
	if(eeprom_read_byte(MODE_addr)){
		setScrnAndPWM(brightness);
		setMode(MAN);
	}
	else{
		setMode(DMX);
	}
}

void setPermanentMode(bool temp){
	if(blinkTimeoutIgnore != temp){
		if(temp){
			LCD_PRINT("P", 15);
		}
		else{
			LCD_PRINT(" ", 15);
		}
		blinkTimeoutIgnore = temp;
	}
}

void setEncMode(encoderMode tempMode){
	switch(tempMode){
		case OFF:
		EncoderMode = OFF;
		TCC0_CTRLA &= ~TC_CLKSEL_DIV1_gc;
		TCC0_INTCTRLA &= ~TC_OVFINTLVL_LO_gc;
		TCC0_INTCTRLB &= ~(TC_CCAINTLVL_LO_gc | TC_CCBINTLVL_LO_gc);
		update_EEPROM_RAM();
		break;

		case MODESEL:
		#define TCC0_CNT_MODE (int)(160 * MODE_DIV)
		#define TCC0_PER_MODE (int)(320 * MODE_DIV)
		TCC0_CNT = TCC0_CNT_MODE;
		TCC0_CTRLB &= ~TC_WGMODE_DS_B_gc;
		TCC0_PER = TCC0_PER_MODE;
		TCC0_INTCTRLA |= TC_OVFINTLVL_LO_gc;
		OVF_firstSam = true;
		TCC0_INTCTRLB &= ~(TC_CCAINTLVL_LO_gc | TC_CCBINTLVL_LO_gc);
		EncoderMode = MODESEL;
		TCC0_CTRLA |= TC_CLKSEL_DIV1_gc;
		break;

		case CHANSEL:
		#define TCC0_CNT_CHAN (int)(160 * CHANNEL_DIV)
		#define TCC0_PER_CHAN (int)(320 * CHANNEL_DIV)
		TCC0_CNT = TCC0_CNT_CHAN;
		TCC0_CTRLB &= ~TC_WGMODE_DS_B_gc;
		TCC0_PER = TCC0_PER_CHAN;
		TCC0_INTCTRLA |= TC_OVFINTLVL_LO_gc;
		OVF_firstSam = true;
		TCC0_INTCTRLB &= ~(TC_CCAINTLVL_LO_gc | TC_CCBINTLVL_LO_gc);
		EncoderMode = CHANSEL;
		TCC0_CTRLA |= TC_CLKSEL_DIV1_gc;
		break;

		case BRIGHTSEL:
		EncoderMode = BRIGHTSEL;
		TCC0_CTRLA |= TC_CLKSEL_DIV1_gc;
		TCC0_CTRLB |= TC_WGMODE_DS_B_gc;
		TCC0_INTCTRLA &= ~TC_OVFINTLVL_LO_gc;
		TCC0_INTCTRLB &= ~(TC_CCAINTLVL_LO_gc | TC_CCBINTLVL_LO_gc);
		#define TCC0_PER_BRIGHTNESS (3080 * BRIGHTNESS_DIV)
		TCC0_PER = TCC0_PER_BRIGHTNESS;
		TCC0_CNT = (int)(brightness * BRIGHTNESS_FINAL_DIV);
		break;
	}
}

void exitEditMode(){
	setEncMode(OFF);
	currBlinkMaskPtr = NULLBlinkVal;
	editMode = false;
	TCD0_PER = BUTTON_DELAY;
	setPermanentMode(false);
}



//All ISR
ISR(TCD0_CCA_vect){
	#ifdef INVERT_BUTTON
		PORTC_PIN2CTRL |= PORT_INVEN_bm; //When the button is released and TCD0 starts counting up again set PC2 direction to normal
	#else
		PORTC_PIN2CTRL &= ~PORT_INVEN_bm; //When the button is released and TCD0 starts counting up again set PC2 direction to normal
    #endif
	
	TCD0_INTCTRLB &= ~TC_CCAINTLVL_LO_gc;
} 

ISR(TCD0_CCB_vect){
	TCD0_PER = 1000; //Set debounce timer for short press detection
	longPressDetection = false;
	TCD0_INTCTRLB &= ~TC_CCBINTLVL_LO_gc; //Disable interrupt
}

void buttonRegularResponse(){
	if(!editMode){
		TCD0_PER = 1000; //Set debounce timer for short press detection
		editMode = true;
		currentSelector = 0;
		editCounter = 0;
	}
	if(currentMode == DMX){
		switch(currentSelector){
			case 0:
			currBlinkMaskPtr = DMXBlinkVal0;
			setEncMode(MODESEL);
			break;
			case 1:
			currBlinkMaskPtr = DMXBlinkVal1;
			dmxMult = 100;
			setEncMode(CHANSEL);
			break;
			case 2:
			currBlinkMaskPtr = DMXBlinkVal2;
			dmxMult = 10;
			break;
			case 3:
			currBlinkMaskPtr = DMXBlinkVal3;
			dmxMult = 1;
			break;
			case 4:
			exitEditMode();
			break;
		}
	}
	if(currentMode == MAN){
		switch(currentSelector){
			case 0:
			currBlinkMaskPtr = MANBlinkVal0;
			setEncMode(MODESEL);
			break;
			case 1:
			TCD0_CNT = 1;
			TCD0_PER = 46875;
			TCD0_CCB = 0;
			TCD0_INTFLAGS |= (1 << 5); //Clear flag because the timer was just at 0
			longPressDetection = true;
			TCD0_INTCTRLB |= TC_CCBINTLVL_LO_gc; //When the button is released before the longPress mark, return to normal operation
			currBlinkMaskPtr = MANBlinkVal1;
			setEncMode(BRIGHTSEL);
			break;
			case 2:
			exitEditMode();
			break;
		}
	}
	currentSelector++;
}

//PC2 interrupt when button is pressed
ISR(TCD0_OVF_vect){
	
	if(longPressDetection){
		setPermanentMode(true);
		
		TCD0_INTCTRLB &= ~TC_CCBINTLVL_LO_gc;
		longPressDetection = false;
		//Short press detection again
		TCD0_PER = 1000;
	}
	else{
		buttonRegularResponse();
	}
	if(!longPressDetection){
		#ifdef INVERT_BUTTON
			PORTC_PIN2CTRL &= ~PORT_INVEN_bm; //Invert button behaviour
		#else
			PORTC_PIN2CTRL |= PORT_INVEN_bm;
		#endif
		TCD0_INTCTRLB |= TC_CCAINTLVL_LO_gc; //Setup compare interrupt so input will be un-inverted after release
	}
	
}

ISR(TCC0_OVF_vect){
	if(!OVF_firstSam){
		if(EncoderMode == CHANSEL){
			if(DMXChan - dmxMult >= DMXmin && TCC0_CNT >= TCC0_CNT_CHAN){
				DMXChan -= dmxMult;
				LCD_PRINTDEC(DMXChan, 22, 3);
				LCD_PRINTDEC(DMXChan + 1, 28, 3);
			}
			else if(DMXChan + dmxMult <= DMXMax && TCC0_CNT < TCC0_CNT_CHAN){
				DMXChan += dmxMult;
				LCD_PRINTDEC(DMXChan, 22, 3);
				LCD_PRINTDEC(DMXChan + 1, 28, 3);
			}
			TCC0_CNT = TCC0_CNT_CHAN;
		}
		else{
			
			if(TCC0_CNT < TCC0_CNT_MODE){
				setMode(DMX);
			}
			else{
				setMode(MAN);
			}
			TCC0_CNT = TCC0_CNT_MODE;
		}

	}
	else{
		
		OVF_firstSam = false;
	}
	
}

ISR(RTC_OVF_vect){
	
	if(blinkTimeoutIgnore){
		editCounter = 0;
	}
	else
	{
		editCounter++;
	}
	if(editCounter > 15){
		longPressDetection = false;
		TCD0_INTCTRLB &= ~TC_CCBINTLVL_LO_gc; //Disable interrupt
		exitEditMode();
	}
}

ISR(USARTD0_RXC_vect){//Interrupt for new DMX char
	bool FERR_flag = USARTD0_STATUS & USART_FERR_bm;
	uint16_t USART_data = USARTD0_DATA;
	if(!FERR_flag){
		if(cnt == DMXChan){//If DMX channel matches the set DMX channel
			finalRes = USART_data << 8; //Buffer 8 MSB
			//LCD_PRINTDEC(USART_data, 16, 5);
		}
		if(cnt == DMXChan + 1){//If DMX channel matches the set DMX channel + 1
			//LCD_PRINTDEC(USART_data, 0, 5);
			finalRes |= USART_data; //Buffer LSB
			TCE0_CCABUF = finalRes >> 2; //Set compare register for PWM in 14 bits to make PWM frequency higher
		
			lt = true;
		}

		if(DMXErrFlag){
			lt = true;
			if(cnt == 0 && USART_data == 0){
				DMXErrFlag = false;
				LCD_PRINT("      ", 10);
				lt = true;
			}
		}
		DMXErrCnt = 0;
		cnt++;//Increment channel counter
	}
	else{
		cnt = 0;
	}
}

ISR(TCC1_OVF_vect){
	DMXErrCnt++;
	if((DMXErrCnt > 11494) && !DMXErrFlag){ //If there hasn't been any change in 1 second
		DMXErrFlag = true;
		LCD_PRINT("NO DMX", 10);
		lt = true;
		DMXErrCnt = 0;
	}
	
}

int main(void)
{
	PORTC_DIRSET = (1 << MOSI_PIN) | (1 << SS_PIN) | (1 << SCK_PIN) | (1 << RS_PIN); //Set all pins used for communication to output
	clk_set_32MHz();
	_delay_ms(70);
	setup_PWM();
	update_RAM_EEPROM();
    setup_SPI();
    _delay_ms(100);
    setup_disp();
	setup_btn();
	QDEC_INIT();
	setup_blink();
	DMX_init();
	updateDisp();
	setup_int();
    while (1) 
    {
		if(editMode){ //If the display is in edit mode update the display
			if(EncoderMode == BRIGHTSEL){
				updateBrightness();
			}
			updateDisp();
			lt = true;
		}
		else{ 
			if (lt) //If lt is altered by another factor update the display as well
			{
				updateDisp();
				lt = false;
			}
		}
		_delay_ms(20);
    }
	
}