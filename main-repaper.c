#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_spi.h"

#define LINES_PER_DISPLAY 96
#define DOTS_PER_LINE 128
#define BYTES_PER_LINE 16

extern unsigned char venus_1_44_bits[];
extern unsigned char aphrodite_1_44_bits[];

struct frame_buffer
{
  unsigned char data[BYTES_PER_LINE*LINES_PER_DISPLAY];
};

struct frame_buffer fb1, fb2;

typedef enum {           // Image pixel -> Display pixel
	EPD_compensate,  // B -> W, W -> B (Current Image)
	EPD_white,       // B -> N, W -> W (Current Image)
	EPD_inverse,     // B -> N, W -> B (New Image)
	EPD_normal       // B -> B, W -> W (New Image)
} EPD_stage;

#define STACK_TOP 0x20010000

static void reset_handler(void);

unsigned int vector_table[2]  __attribute__ ((section("vectors")))= {
  (unsigned int) STACK_TOP,	// Stack pointer
  (unsigned int) reset_handler	// Reset handler
};

extern unsigned int _start_bss, _end_bss;
extern unsigned int _start_idata, _end_idata, _end_text;

static void reset_handler(void)
{
  unsigned int *p1,*p2;

  for (p1 = &_end_text, p2 = &_start_idata; p2 < &_end_idata; p1++,p2++)
    *p2 = *p1;

  for (p1 = &_start_bss; p1 < &_end_bss; p1++)
   *p1 = 0;

  SystemInit();

  main();
}

// Free GPIO - PA1, PA2, PA3, PA15, PC1, PB14
// PA9 - Green LED (TIM1_CH2)
// PA8 - PWM (TIM1_CH1)
// PB12 - SPI2_NSS - EPD_CS 19 (Brown)
// PB13 - SPI2_SCK - SPI_CLK 7 (Yellow)
// PB15 - SPI2_MOSI - SPI_MOSI 15 (Blue) 
// PD14 - Red LED
// PC2 - 8 BUSY (Orange)
// PA1 - 10 RESET (Black) 
// PA2 - 11 PANEL_ON (Red)
// PA3 - 12 DISCHARGE (White)
// PA15 - 13 BORDER_CONTROL (Grey)

#define RED_LED_PIN GPIO_Pin_14
#define RED_LED_GPIO GPIOD

#define BUSY_PIN GPIO_Pin_2
#define BUSY_GPIO GPIOC

#define RESET_PIN GPIO_Pin_1
#define RESET_GPIO GPIOA

#define PANEL_ON_PIN GPIO_Pin_2
#define PANEL_ON_GPIO GPIOA

#define DISCHARGE_PIN GPIO_Pin_3
#define DISCHARGE_GPIO GPIOA

#define BORDER_CONTROL_PIN GPIO_Pin_15
#define BORDER_CONTROL_GPIO GPIOA

#define EPD_CS_PIN GPIO_Pin_12
#define EPD_CS_GPIO GPIOB
 
static void delay_us(int us)
{
  int i;

  for (i=0; i<us*10; i++)
    ;
}

static void delay_ms(int ms)
{
  delay_us(1000*ms);
}

static void cs_assert(void)
{
  GPIO_SetBits(EPD_CS_GPIO, EPD_CS_PIN);  
}

static void cs_deassert(void)
{
  GPIO_ResetBits(EPD_CS_GPIO, EPD_CS_PIN);  
}

void epd_wait_busy()
{
  GPIO_SetBits(RED_LED_GPIO, RED_LED_PIN);  
  while (GPIO_ReadInputDataBit(BUSY_GPIO, BUSY_PIN))
    ;
  delay_us(10);
  GPIO_ResetBits(RED_LED_GPIO, RED_LED_PIN);  
}

static void cs_strobe() {
  cs_assert();
  delay_us(10);
  cs_deassert();
}

static void spi_write(unsigned char c)
{
  while (!SPI2->SR && SPI_I2S_FLAG_TXE);
  SPI2->DR = c;

  delay_us(2);
}

static void spi_writebuffer(unsigned char * c, int len)
{
  int i;

  for (i=0; i<len; i++)
    spi_write(c[i]);
}

static void spi_writecommand(unsigned char reg, unsigned char * data, unsigned char len)
{
  cs_strobe();

  spi_write(0x70);
  spi_write(reg);

  cs_strobe();

  spi_write(0x72);
  spi_writebuffer(data,len);

  cs_assert();
}

static void spi_writesingle(unsigned char c, unsigned char d)
{
  unsigned char buffer[1];

  buffer[0] = d;
  spi_writecommand(c, buffer, 1);
}

static void spi_writeheader(unsigned char reg)
{
  cs_strobe();

  spi_write(0x70);
  spi_write(reg);

  cs_assert();
}

static void epd_line(int line, unsigned char * data, int stage)
{
  int i;

  spi_writesingle(0x04, 0x03);

  spi_writeheader(0x0a);
  delay_us(10);
  cs_deassert();
  spi_write(0x72);

  for (i = BYTES_PER_LINE-1; i >= 0; i --)
  {
    unsigned char pixels = data[i] & 0xaa;

    switch(stage) {
      case EPD_compensate:  // B -> W, W -> B (Current Image)
        pixels = 0xaa | ((pixels ^ 0xaa) >> 1);
        break;
      case EPD_white:       // B -> N, W -> W (Current Image)
        pixels = 0x55 + ((pixels ^ 0xaa) >> 1);
        break;
      case EPD_inverse:     // B -> N, W -> B (New Image)
        pixels = 0x55 | (pixels ^ 0xaa);
        break;
      case EPD_normal:       // B -> B, W -> W (New Image)
        pixels = 0xaa | (pixels >> 1);
        break;
      }
    spi_write(pixels);
    epd_wait_busy();
  }

  for (i = 0; i < LINES_PER_DISPLAY/4; i++)
  {
    if (line / 4 == i) 
      spi_write(0xc0 >> (2 * (line & 0x03)));
    else
      spi_write(0x00);
    epd_wait_busy();
  }

  for (i = 0; i < BYTES_PER_LINE; i++)
  {
    unsigned char pixels = data[i] & 0x55;

    switch(stage)
    {
      case EPD_compensate:  // B -> W, W -> B (Current Image)
        pixels = 0xaa | (pixels ^ 0x55);
        break;
      case EPD_white:       // B -> N, W -> W (Current Image)
        pixels = 0x55 + (pixels ^ 0x55);
        break;
      case EPD_inverse:     // B -> N, W -> B (New Image)
        pixels = 0x55 | ((pixels ^ 0x55) << 1);
        break;
      case EPD_normal:       // B -> B, W -> W (New Image)
        pixels = 0xaa | pixels;
        break;
    }

     unsigned char p1 = (pixels >> 6) & 0x03;
     unsigned char p2 = (pixels >> 4) & 0x03;
     unsigned char p3 = (pixels >> 2) & 0x03;
     unsigned char p4 = (pixels >> 0) & 0x03;
     pixels = (p1 << 0) | (p2 << 2) | (p3 << 4) | (p4 << 6);
  
    spi_write(pixels);
    epd_wait_busy();
  }

  cs_assert();
  spi_writesingle(0x02, 0x2f);
}

static void epd_line_fixed(int line, int value, int scan_byte)
{
  int i;

  spi_writesingle(0x04, 0x03);

  spi_writeheader(0x0a);
  delay_us(10);
  cs_deassert();
  spi_write(0x72);

  for (i = 0; i < BYTES_PER_LINE; i++)
  {
    spi_write(value);
    epd_wait_busy();
  }

  for (i = 0; i < LINES_PER_DISPLAY/4; ++i)
  {
    if (scan_byte && (line / 4 == i))
      spi_write(0xc0 >> (2 * (line & 0x03)));
    else
      spi_write(0x00);
    epd_wait_busy();
  }

  for (i = 0; i < BYTES_PER_LINE; i++)
  {
    spi_write(value);
    epd_wait_busy();
  }

  cs_assert();
  spi_writesingle(0x02, 0x2f);
}

void epd_frame_fixed(unsigned char fixed_value, int repeat_count)
{
  int line;
  int i;

  for (i=0; i<repeat_count; i++)
    for (line = 0; line < LINES_PER_DISPLAY; line++)
      epd_line_fixed(line, fixed_value, 1);
}

void epd_frame_data(unsigned char * data, int repeat_count, EPD_stage stage)
{
  int line;
  int i;

  for (i=0; i<repeat_count; i++)
    for (line = 0; line < LINES_PER_DISPLAY; line++)
      epd_line(line, data + line * BYTES_PER_LINE, stage);
}

void epd_clear()
{
  epd_frame_fixed(0xFF, 10);
  epd_frame_fixed(0xAA, 10);
}

void epd_image(struct frame_buffer * old, struct frame_buffer * new)
{
  epd_frame_fixed(0xAA, 10);

  // Draw new image
  epd_frame_data(new->data, 10, EPD_inverse);
  epd_frame_data(new->data, 10, EPD_normal);
}

void epd_power_on(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_ResetBits(RESET_GPIO, RESET_PIN);
  GPIO_ResetBits(PANEL_ON_GPIO, PANEL_ON_PIN);
  GPIO_ResetBits(DISCHARGE_GPIO, DISCHARGE_PIN);
  GPIO_ResetBits(BORDER_CONTROL_GPIO, BORDER_CONTROL_PIN);
  GPIO_ResetBits(EPD_CS_GPIO, EPD_CS_PIN);
  
  TIM_Cmd(TIM1, ENABLE);

  delay_ms(5);

  GPIO_SetBits(PANEL_ON_GPIO, PANEL_ON_PIN);

  delay_ms(10);

  GPIO_SetBits(RESET_GPIO, RESET_PIN);
  GPIO_SetBits(BORDER_CONTROL_GPIO, BORDER_CONTROL_PIN);
  GPIO_SetBits(EPD_CS_GPIO, EPD_CS_PIN);

  delay_ms(5);

  GPIO_ResetBits(RESET_GPIO, RESET_PIN);
  delay_ms(5);

  GPIO_SetBits(RESET_GPIO, RESET_PIN);
  delay_ms(5);

  epd_wait_busy();

  unsigned char channel_select[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00};

  spi_writecommand(0x01, channel_select, 8);  

  // DC/DC frequency
  spi_writesingle(0x06, 0xFF);

  // high power mode osc
  spi_writesingle(0x07, 0x9D);

  // disable ADC
  spi_writesingle(0x08, 0x00);

  unsigned char vcom_level[] = {0xd0, 0x00};
  spi_writecommand(0x09, vcom_level, 2);

  spi_writesingle(0x04, 0x03);

  delay_ms(5);

  spi_writesingle(0x03, 0x01);
  spi_writesingle(0x03, 0x00);

  delay_ms(5);

  spi_writesingle(0x05, 0x01);

  delay_ms(30);

  // Disable PWM

  TIM_Cmd(TIM1, DISABLE);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOA, GPIO_Pin_8);

  spi_writesingle(0x05, 0x0f);

  delay_ms(30);

  spi_writesingle(0x02, 0x24);
}

void epd_power_off(void)
{
  int i;

  // Nothing frame
  epd_frame_fixed(0x55, 1);

  // Dummy line
  epd_line_fixed(0, 0x55, 0);

  delay_ms(25);

  GPIO_ResetBits(BORDER_CONTROL_GPIO, BORDER_CONTROL_PIN);

  delay_ms(300);

  // Latch reset turn on
  spi_writesingle(0x03, 0x01);

  // Output enable off  
  spi_writesingle(0x02, 0x05);

  // Power off
  spi_writesingle(0x05, 0x0e);

  // Power off charge pump
  spi_writesingle(0x05, 0x02);

  // Discharge
  spi_writesingle(0x04, 0x0c);

  delay_ms(120);

  // Turn off chrage pumps
  spi_writesingle(0x05, 0x00);

  // Turn off osc
  spi_writesingle(0x07, 0x0d);

  // Discharge internal
  spi_writesingle(0x04, 0x50);

  delay_ms(40);

  // Discharge internal
  spi_writesingle(0x04, 0xa0);

  delay_ms(40);

  // Discharge internal
  spi_writesingle(0x04, 0x00);

  // All signals 0
  GPIO_ResetBits(RESET_GPIO, RESET_PIN);
  GPIO_ResetBits(PANEL_ON_GPIO, PANEL_ON_PIN);
  GPIO_ResetBits(BORDER_CONTROL_GPIO, BORDER_CONTROL_PIN);
  GPIO_ResetBits(EPD_CS_GPIO, EPD_CS_PIN);

  // External discharge
  GPIO_SetBits(DISCHARGE_GPIO, DISCHARGE_PIN);

  delay_ms(150);

  GPIO_ResetBits(DISCHARGE_GPIO, DISCHARGE_PIN);
}

void setup_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;
  unsigned long PrescalerValue;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  unsigned short period, pulse;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);

  // Configure SPI pins - PB13 SPI2_SCK, PB15 SPI2_MISO
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

  // Configure SPI pins - PB12 SPI2_NSS
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure the SPI bus
  SPI_StructInit(&SPI_InitStructure);
  SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI2, &SPI_InitStructure);

  SPI_Cmd(SPI2, ENABLE);

  // Compute the value for the ARR register to have a period of 200 KHz
  // 50% duty cycle
  period = (SystemCoreClock / 200000 ) - 1;	
  pulse = (uint16_t) (((uint32_t) 50 * (period - 1)) / 100);

  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM1, ENABLE);

  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  TIM_Cmd(TIM1, DISABLE);
}

int main(void)
{
  int i;
  int line;
  int count;

  setup_gpio();

  epd_power_on();

  epd_clear();

  for (i=0; i<BYTES_PER_LINE * LINES_PER_DISPLAY; i++)
    fb1.data[i] = venus_1_44_bits[i];
  for (i=0; i<BYTES_PER_LINE * LINES_PER_DISPLAY; i++)
    fb2.data[i] = aphrodite_1_44_bits[i];

  epd_image(0, &fb1);

  delay_ms(1000);

  epd_image(&fb1, &fb2);

  epd_power_off();

  while (1)
  {
  }
}
