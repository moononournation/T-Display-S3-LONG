/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/
#include <U8g2lib.h>
#include <Arduino_GFX_Library.h>
#define GFX_DEV_DEVICE LILYGO_T_DISPLAY_S3_LONG
#define GFX_BL 1
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    12 /* cs */, 17 /* sck */, 13 /* d0 */, 18 /* d1 */, 21 /* d2 */, 14 /* d3 */);
Arduino_GFX *g = new Arduino_AXS15231B(bus, 47 /* RST */, 0 /* rotation */, false /* IPS */, 180 /* width */, 640 /* height */);
#define CANVAS
Arduino_Canvas *gfx = new Arduino_Canvas(180 /* width */, 640 /* height */, g, 0 /* output_x */, 0 /* output_y */, 1 /* rotation */);
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

/* more fonts at: https://github.com/moononournation/ArduinoFreeFontFile.git */

#define BUTTON_PIN 0
#define INTERRUPT_PIN 11

char buf[16];

unsigned long start_time;
bool timing = false;
void IRAM_ATTR ISR()
{
  if (timing)
  {
    timing = false;
  }
  else
  {
    start_time = millis();
    timing = true;
  }
}

void setup(void)
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX U8g2 Font Hello World example");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  ledcSetup(1, 12000, 8);   // 12 kHz PWM, 8-bit resolution
  ledcAttachPin(GFX_BL, 1); // assign RGB led pins to channels
  ledcWrite(1, 63);
#endif

  attachInterrupt(BUTTON_PIN, ISR, FALLING);
  attachInterrupt(INTERRUPT_PIN, ISR, FALLING);

  gfx->setTextColor(GREEN);
  gfx->setFont(u8g2_font_logisoso92_tn);

  gfx->setCursor(10, 136);
  gfx->print("00:00:00.000");
  gfx->flush();
}

void loop()
{
  if (timing)
  {
    unsigned long time = millis() - start_time;

    int ss = time % 1000;
    time /= 1000;
    int sec = time % 60;
    time /= 60;
    int min = time % 60;
    time /= 60;
    sprintf(buf, "%.2d:%.2d:%.2d.%.3d", time, min, sec, ss);

    gfx->fillScreen(BLACK);

    gfx->setCursor(10, 136);
    gfx->print(buf);
    gfx->flush();
  }
  else
  {
    delay(1);
  }
}
