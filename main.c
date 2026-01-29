#include <hidef.h>      /* basic macros */
#include "derivative.h" /* chip specific stuff */

/*
   Project: Ball and Beam (safe/simple version)
   Board: Dragon12-JR (MC9S12DG256)
   Clock: 4MHz (PLL off on purpose to avoid issues)
   UART Frame: [0xFF, TopDist, BottomDist]
*/

// Timer math at 4MHz:
// Bus cycle = 250ns
// Prescaler 4 -> timer ticks every 1us
// Speed of sound ~340 m/s
// That’s ~29us per cm one-way, ~58us round trip
// So ~58 timer counts per cm
#define COUNTS_PER_CM 58 

// --- Function declarations ---
void SCI1_Init_4MHz(void); 
void Timer_Init_4MHz(void);
void SCI1_Tx(unsigned char data);
void Delay_ms(unsigned int ms);
void Delay_Trigger(void);
unsigned int Read_Sensor(unsigned char sensorID);
void Show_7Seg(unsigned char num);

// 7-seg codes (common anode)
// index = number to display
const unsigned char seg7_table[] = { 
    0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90 
};

void main(void) {
    unsigned int distTop, distBottom;
    
    // Always kill watchdog first or it will reset the MCU
    COPCTL = 0x00; 

    // No PLL setup -> stay at default 4MHz
    
    // Init serial + timer assuming 4MHz
    SCI1_Init_4MHz();      
    Timer_Init_4MHz();     
    
    // Port directions
    DDRB = 0xFF; PORTB = 0x00; // LEDs
    DDRH = 0xFF; PTH = 0xFF;   // 7-seg
    DDRT = 0x05;               // PT0 & PT2 as trigger outputs

    EnableInterrupts;

    for(;;) {
        // Read upper ultrasonic sensor
        distTop = Read_Sensor(1);
        Delay_ms(40); 
        
        // Read lower ultrasonic sensor
        distBottom = Read_Sensor(2);
        Delay_ms(40); 
        
        // Limit values so they fit in one byte
        if (distTop > 254) distTop = 254;
        if (distBottom > 254) distBottom = 254;
        
        // Send data to Arduino
        SCI1_Tx(0xFF);                  // Start byte
        SCI1_Tx((unsigned char)distTop);
        SCI1_Tx((unsigned char)distBottom);
        
        // Debug / feedback stuff
        PORTB = ~PORTB;                 // Toggle LEDs
        Show_7Seg((unsigned char)distTop % 10); // Show last digit
    }
}

// --- ULTRASONIC SENSOR READ (timer capture based) ---
unsigned int Read_Sensor(unsigned char sensorID) {
    unsigned int risingEdge, fallingEdge, pulseWidth;
    unsigned long timeout = 0;
    
    if(sensorID == 1) { // Sensor 1: PT0 trig, PT1 echo
        PTT_PTT0 = 1; 
        Delay_Trigger(); 
        PTT_PTT0 = 0;
        
        // Capture rising edge
        TFLG1 = 0x02; 
        TCTL4 = (TCTL4 & 0xF3) | 0x04;
        while(!(TFLG1 & 0x02)) { 
            if(++timeout > 30000) return 254; 
        }
        risingEdge = TC1; 
        
        // Capture falling edge
        TFLG1 = 0x02; 
        TCTL4 = (TCTL4 & 0xF3) | 0x08;
        timeout = 0;
        while(!(TFLG1 & 0x02)) { 
            if(++timeout > 30000) return 254; 
        }
        fallingEdge = TC1;
        
    } else { // Sensor 2: PT2 trig, PT3 echo
        PTT_PTT2 = 1; 
        Delay_Trigger(); 
        PTT_PTT2 = 0;
        
        TFLG1 = 0x08; 
        TCTL4 = (TCTL4 & 0x3F) | 0x40;
        while(!(TFLG1 & 0x08)) { 
            if(++timeout > 30000) return 254; 
        }
        risingEdge = TC3;
        
        TFLG1 = 0x08;
        TCTL4 = (TCTL4 & 0x3F) | 0x80;
        timeout = 0;
        while(!(TFLG1 & 0x08)) { 
            if(++timeout > 30000) return 254; 
        }
        fallingEdge = TC3;
    }
    
    // Handle normal case vs timer overflow
    if (fallingEdge >= risingEdge) 
        pulseWidth = fallingEdge - risingEdge;
    else 
        pulseWidth = (0xFFFF - risingEdge) + fallingEdge + 1;

    // Timer tick = 1us
    // About 58us per cm
    return (pulseWidth / 58);
}

// --- INIT CODE (4MHz CONFIG) ---

void SCI1_Init_4MHz(void) {
    // 4MHz bus, target baud = 9600
    // Divider = 26 (close enough)
    SCI1BD = 26; 
    SCI1CR1 = 0x00; 
    SCI1CR2 = 0x08; // Enable TX only
}

void Timer_Init_4MHz(void) {
    TSCR1 = 0x80; // Turn timer on
    // Prescaler = 4 -> 1MHz timer clock
    TSCR2 = 0x02; 
    TIOS &= ~0x0F; // All input capture
}

void SCI1_Tx(unsigned char data) {
    // Wait until transmit buffer is empty
    while(!(SCI1SR1 & 0x80)); 
    SCI1DRL = data;
}

// --- DELAY ROUTINES ---

void Delay_Trigger(void) {
    // HC-SR04 needs at least 10us trigger pulse
    volatile unsigned char i;
    for(i=0; i<15; i++); 
}

void Delay_ms(unsigned int ms) {
    volatile unsigned int i, j;
    // Crude delay loop, not super accurate
    for(i=0; i<ms; i++)
        for(j=0; j<600; j++); 
}

void Show_7Seg(unsigned char num) {
    // Blank display if number is invalid
    if (num > 9) PTH = 0xFF;
    else PTH = seg7_table[num];
}
