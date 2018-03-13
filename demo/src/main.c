//*****************************************************************************
//*   A demo example using several of the peripherals on the base board
//*
//*   Copyright(C) 2011, EE2024
//*   All rights reserved.
//*
//******************************************************************************/
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "stdbool.h" 
#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "temp.h"
#include "led7seg.h"
#include "light.h"
#include "lpc17xx_uart.h" 
static uint8_t barPos = 2;
unsigned int d = 0,k = 0;
unsigned int f = 0,g = 0;
int8_t nextstate = -1;
int8_t state = 0;
uint32_t initial = 0;
uint32_t final = 0;
uint32_t my_light_value = 0;
uint32_t myval = 0;

static void moveBar(uint8_t steps, uint8_t dir) {
uint16_t ledOn = 0;
if (barPos == 0)
ledOn = (1 << 0) | (3 << 14);
else if (barPos == 1)
ledOn = (3 << 0) | (1 << 15);
else
ledOn = 0x07 << (barPos-2);
barPos += (dir*steps);
barPos = (barPos % 16);
pca9532_setLeds(ledOn, 0xffff);
}

static void drawOled(uint8_t joyState)
{
static int wait = 0;
static uint8_t currX = 48;
static uint8_t currY = 32;
static uint8_t lastX = 0;
static uint8_t lastY = 0;
if ((joyState & JOYSTICK_CENTER) != 0) {
oled_clearScreen(OLED_COLOR_BLACK);
return;
}
if (wait++ < 3)
return;
wait = 0;
if ((joyState & JOYSTICK_UP) != 0 && currY > 0) {
currY--;
}
if ((joyState & JOYSTICK_DOWN) != 0 && currY < OLED_DISPLAY_HEIGHT-1) {
currY++;
}
if ((joyState & JOYSTICK_RIGHT) != 0 && currX < OLED_DISPLAY_WIDTH-1) {
currX++;
}
if ((joyState & JOYSTICK_LEFT) != 0 && currX > 0) {
currX--;
}
if (lastX != currX || lastY != currY) {
oled_putPixel(currX, currY, OLED_COLOR_WHITE);
lastX = currX;
lastY = currY;
}
}

static void init_ssp(void)
{
SSP_CFG_Type SSP_ConfigStruct;
PINSEL_CFG_Type PinCfg;
/*
* Initialize SPI pin connect
* P0.7 - SCK;
* P0.8 - MISO
* P0.9 - MOSI
* P2.2 - SSEL - used as GPIO
*/
PinCfg.Funcnum = 2;
PinCfg.OpenDrain = 0;
PinCfg.Pinmode = 0;
PinCfg.Portnum = 0;
PinCfg.Pinnum = 7;
PINSEL_ConfigPin(&PinCfg);
PinCfg.Pinnum = 8;
PINSEL_ConfigPin(&PinCfg);
PinCfg.Pinnum = 9;
PINSEL_ConfigPin(&PinCfg);
PinCfg.Funcnum = 0;
PinCfg.Portnum = 2;
PinCfg.Pinnum = 2;
PINSEL_ConfigPin(&PinCfg);
SSP_ConfigStructInit(&SSP_ConfigStruct);
// Initialize SSP peripheral with parameter given in structure above
SSP_Init(LPC_SSP1, &SSP_ConfigStruct);
// Enable SSP peripheral
SSP_Cmd(LPC_SSP1, ENABLE);
}

static void init_i2c(void)
{
PINSEL_CFG_Type PinCfg;
/* Initialize I2C2 pin connect */
PinCfg.Funcnum = 2;
PinCfg.Pinnum = 10;
PinCfg.Portnum = 0;
PINSEL_ConfigPin(&PinCfg);
PinCfg.Pinnum = 11;
PINSEL_ConfigPin(&PinCfg);
// Initialize I2C peripheral
I2C_Init(LPC_I2C2, 100000);
/* Enable I2C1 operation */
I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void)
{
// Initialize button
PINSEL_CFG_Type PinCfg;
PinCfg.Funcnum = 0;
PinCfg.Portnum = 2;
PinCfg.Pinnum = 10;
PINSEL_ConfigPin(&PinCfg);
GPIO_SetDir(2, 1<<10, 0);
PinCfg.Funcnum = 0;
PinCfg.Portnum = 1;
PinCfg.Pinnum = 31;
PINSEL_ConfigPin(&PinCfg);
GPIO_SetDir(1, 1<<31, 0);
PinCfg.Funcnum = 0;
PinCfg.Portnum = 0;
PinCfg.Pinnum = 6;
PINSEL_ConfigPin(&PinCfg);
GPIO_SetDir(0, 1<<6, 0);
PinCfg.Funcnum = 0;
PinCfg.Portnum = 0;
PinCfg.Pinnum = 3;
PINSEL_ConfigPin(&PinCfg);
GPIO_SetDir(0, 1<<3,0); // accelerometer interrupt pin
PinCfg.Funcnum = 0;
PinCfg.Portnum = 2;
PinCfg.Pinnum = 5;
PINSEL_ConfigPin(&PinCfg);
GPIO_SetDir(2,1<<5,0);
PinCfg.Funcnum = 0;
PinCfg.Portnum = 2;
PinCfg.Pinnum = 1;
PINSEL_ConfigPin(&PinCfg);
GPIO_SetDir(2,1<<1,0);
PinCfg.Funcnum = 1;
PinCfg.Portnum = 2;
PinCfg.Pinnum = 13;
PINSEL_ConfigPin(&PinCfg);
GPIO_ClearValue(2,(1<<13));
}

volatile uint32_t msTicks;
volatile uint32_t msTicks2;
void SysTick_Handler(void) {
msTicks++;
msTicks2++;
}

uint32_t getTicks(){
return msTicks;

};
uint32_t getTicks2(){
return msTicks2;
};

void TIMER0_IRQHandler(void)
{
unsigned int isrMask;
isrMask = LPC_TIM0->IR;
LPC_TIM0->IR = 1;
if (d == 0) {
d = 1;
}else{
d = 0;
}
if ((state == 2) && (my_light_value < 100)) {
rgb_setLeds(RGB_BLUE,d);
f = 1;
} else {
rgb_setLeds(RGB_BLUE,0);
f = 0;
}
if ((state == 1) && (myval >= 280)) {
rgb_setLeds(RGB_RED,d);
g = 1;
} else {
g = 0;
rgb_setLeds(RGB_RED,0);
}
}

letters (uint32_t i) {
uint8_t array[] = {
0x24,0xAF,0xE0,0xA2,0x2B,0x32,0x30,0xA7,0x20,0x22,
0x21,0x38,0x74,0xA8,0x70,0x71,0x10,0x29,0x8F,0xAC,
0xFF,0x7C,0xFF,0xB9,0x04,0x61,0x03,0xF9,0x12,0x78
};
uint32_t b = 0;
b = array[i];
return b;
}
void pinsel_uart3(void){
PINSEL_CFG_Type PinCfg;
PinCfg.Funcnum = 2;
PinCfg.Pinnum = 0;
PinCfg.Portnum = 0;
PINSEL_ConfigPin(&PinCfg);
PinCfg.Pinnum = 1;
PINSEL_ConfigPin(&PinCfg);
}
void init_uart(void){
UART_CFG_Type uartCfg;
uartCfg.Baud_rate = 115200;
uartCfg.Databits = UART_DATABIT_8;
uartCfg.Parity = UART_PARITY_NONE;
uartCfg.Stopbits = UART_STOPBIT_1;
//pin select for uart3;
pinsel_uart3();
//supply power & setup working parameters for uart3
UART_Init(LPC_UART3, &uartCfg);
//enable transmit for uart3
UART_TxCmd(LPC_UART3, ENABLE);
}

static char* msg = NULL;
unsigned int getPrescalarForUs(uint8_t timerPclkBit)

{
unsigned int pclk,prescalarForUs;
pclk = (LPC_SC->PCLKSEL0 >> timerPclkBit) & 0x03; /* get the pclk info for required timer */
switch (pclk)
{
case 0x00:
pclk = SystemCoreClock/4;
break;
case 0x01:
pclk = SystemCoreClock;
break;
case 0x02:
pclk = SystemCoreClock/2;
break;
case 0x03:
pclk = SystemCoreClock/8;
break;
}
prescalarForUs =pclk/1000000 - 1;
return prescalarForUs;
}

EINT0_IRQHandler(void)
{

LPC_SC->EXTINT = (1<<0);
if (state == 0) {
if (nextstate < 2) {
nextstate++;
}} else {
state = 0;
nextstate = 10;
}
LPC_GPIOINT->IO2IntClr |= (1 << 10);
}

EINT3_IRQHandler(void) {
if ((state == 2) && (light_getIrqStatus != 0)) {
light_setLoThreshold(100);
light_clearIrqStatus();
light_getIrqStatus();
}
LPC_GPIOINT->IO2IntClr |= (1 << 5);
}

void all_init (void) {
SysTick_Config(SystemCoreClock/1000);
temp_init(getTicks);
led7seg_init ();
init_i2c();
init_ssp();
init_GPIO();
pca9532_init();
oled_init();
light_init();
SystemInit();
light_enable();
/* ---- Speaker ------> */
GPIO_SetDir(2, 1<<0, 1);
GPIO_SetDir(2, 1<<1, 1);
GPIO_SetDir(0, 1<<27, 1);
GPIO_SetDir(0, 1<<28, 1);
GPIO_SetDir(2, 1<<13, 1);
GPIO_SetDir(0, 1<<26, 1);
GPIO_ClearValue(0, 1<<27); //LM4811-clk
GPIO_ClearValue(0, 1<<28); //LM4811-up/dn
GPIO_ClearValue(2, 1<<13); //LM4811-shutdn
LPC_GPIOINT->IO2IntEnR |= (1 << 10);
LPC_SC->EXTINT = (1<<0);
LPC_PINCON->PINSEL4 = (1<<20);
LPC_SC->EXTMODE = (1<<0);
LPC_SC->EXTPOLAR = (1<<0);
NVIC_EnableIRQ(EINT0_IRQn);
LPC_SC->PCONP |= (1<<1);
LPC_GPIOINT -> IO2IntEnF |= (1<<5);
light_setLoThreshold(100);
light_clearIrqStatus();
light_getIrqStatus();
NVIC_EnableIRQ(EINT3_IRQn);
LPC_TIM0->MCR=(1<<0)|(1<<1);
LPC_TIM0->MR0=(100*1000);
LPC_TIM0->PR=getPrescalarForUs(1);
LPC_TIM0->TCR=(1 << 0);
NVIC_EnableIRQ(TIMER0_IRQn);
NVIC_SetPriorityGrouping(5);
NVIC_SetPriority(EINT0_IRQn, 4);
NVIC_SetPriority(EINT3_IRQn, 5);
NVIC_SetPriority(TIMER0_IRQn, 6);
}

void crashcollision (int32_t x, int32_t y, int32_t z, uint32_t myval) {
uint8_t display_acc[40] = {};
uint8_t display_temp[40] = {};
uint32_t display_state[40] = {};
sprintf(display_state,"-FORWARD-\n");
sprintf(display_acc,"               \n", x, y, z);
sprintf(display_temp,"               \n",(myval/10.0));
oled_putString(5,5,(uint8_t*)display_acc,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(10,20,(uint8_t*)display_temp,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(25,45,(uint8_t*)display_state,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
}
void forwardmode (int32_t X,int32_t Y,int32_t Z,uint32_t myval) {
	uint8_t display_acc[40] = {};
	uint8_t display_temp[40] = {};
	uint32_t display_state[40] = {};
	sprintf(display_state,"-Forward-\n");
	sprintf(display_acc,"x:%d,y:%d,z:%d\n", X, Y, Z);
	sprintf(display_temp,"T:%2.2f       \n",(myval/10.0));
	oled_putString(5,5,(uint8_t*)display_acc,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
	oled_putString(10,20,(uint8_t*)display_temp,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
	oled_putString(25,45,(uint8_t*)display_state,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
}
void stationarymode (int32_t x,int32_t y,int32_t z,uint32_t myval) {
uint8_t display_acc[40] = {};
uint8_t display_temp[40] = {};
uint32_t display_state[40] = {};
sprintf(display_state,"stationary\n");
sprintf(display_acc,"               \n", x, y, z);
sprintf(display_temp,"               \n",(myval/10.0));
oled_putString(5,5,(uint8_t*)display_acc,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(10,20,(uint8_t*)display_temp,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(25,45,(uint8_t*)display_state,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
}

void reversemodenormal (int32_t x, int32_t y, int32_t z, uint32_t myval) {
uint8_t display_acc[40] = {};
uint8_t display_temp[40] = {};
uint32_t display_state[40] = {};
sprintf(display_state,"-Reverse-\n");
sprintf(display_acc,"               \n", x, y, z);
sprintf(display_temp,"               \n",(myval/10.0));
oled_putString(5,5,(uint8_t*)display_acc,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(10,20,(uint8_t*)display_temp,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(25,45,(uint8_t*)display_state,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
}

void reversealertmode (int8_t x, int8_t y, int8_t z) {
uint8_t display_acc[40] = {};
uint8_t display_temp[40] = {};
uint32_t display_state[40] = {};
sprintf(display_state,"-Reverse-\n");
sprintf(display_acc,"               \n", x, y, z);
sprintf(display_temp,"obstacle-nearby\n");
oled_putString(5,5,(uint8_t*)display_acc,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(10,20,(uint8_t*)display_temp,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(25,45,(uint8_t*)display_state,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
}

void temperaturealert (uint32_t myval, int32_t X, int32_t Y, int32_t Z) {
uint8_t display_acc[40] = {};
myval = temp_read();
uint8_t display_temp[40] = {};
uint32_t display_state[40] = {};
sprintf(display_state,"-FORWARD-\n");
sprintf(display_acc,"            \n");
sprintf(display_temp,"HOT: %2.2f\n", (myval/10.0));
oled_putString(5,5,(uint8_t*)display_acc,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(10,20,(uint8_t*)display_temp,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
oled_putString(25,45,(uint8_t*)display_state,OLED_COLOR_BLACK,OLED_COLOR_WHITE);
}

static void changeinstate (void) {
if (nextstate == 0) {
state = state;
}
if ((nextstate == 1) && (state == 0)) {
state = 1;
}
if ((nextstate == 2) && (state == 0)) {
state = 2;
}
}

int main (void) {
all_init();
uint32_t prev2 = 0;
uint32_t curr2 = 0;
prev2 = getTicks();
curr2 = prev2;
int8_t X,Y,Z;
uint8_t display_acc[40] = {};
uint8_t display_temp[40] = {};
uint32_t display_state[40] = {};
int32_t xoff = 0;
int32_t yoff = 0;
int32_t zoff = 0;
int8_t x = 0;
int8_t y = 0;
int8_t z = 0;
uint8_t i = 0;
uint8_t dir = 1;
uint8_t wait = 0;
acc_read(&x, &y, &z);
xoff = 0-x;
yoff = 0-y;
zoff = 64-z;
moveBar(1, dir);
oled_clearScreen(OLED_COLOR_WHITE);
while (1)
{
my_light_value = light_read();
if (nextstate == 10) {
stationarymode(x,y,z,myval);
}
if (f == 1) {
reversealertmode(x,y,z);
}
if (g == 1) {
temperaturealert(myval,X,Y,Z);
}
while ((curr2 - prev2) < 1000) {
curr2 = getTicks();
}
prev2 = getTicks();
acc_read(&x, &y, &z);
myval = temp_read();
changeinstate();
printf("s:%d,n:%d\n",state,nextstate);
if (state == 1) {
forwardmode(X,Y,Z,myval);
}
if (k < 15) {
k++;
if( k== 4 || k == 9 || k == 14) {
X = x;
Y = y;
Z = z;
}
} else {
k = 0;
}
if (state == 2) {
reversemodenormal(x,y,z,myval);
}
if ((state == 0)) {
stationarymode(x,y,z,myval);
state = 0;
}
led7seg_setChar(letters(k), TRUE);
x = x+xoff;
y = y+yoff;
z = z+zoff;
if (y < 0) {
dir = 1;
y = -y;
 }
 else {
dir = -1;
}
if (y > 1 && wait++ > (40 / (1 + (y/10)))) {
moveBar(1, dir);
wait = 0;
}
nextstate = 0;
}
}

void check_failed(uint8_t *file, uint32_t line)
{

while(1);
}
