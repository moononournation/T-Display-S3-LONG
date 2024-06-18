/*******************************************************************************
 * Motion JPEG Image Viewer
 * This is a simple Motion JPEG image viewer example
 *
 * GIFDEC original source:
 * https://gifer.com/en/fxTQ
 * ffmpeg -i "fxTQ.gif" -vf "fps=25,scale=624:624:flags=lanczos,crop=624:180:0:222" -pix_fmt yuvj420p -q:v 9 fxTQ.mjpeg
 *
 * Dependent libraries:
 * JPEGDEC: https://github.com/bitbank2/JPEGDEC.git
 *
 * Setup steps:
 * 1. Change your LCD parameters in Arduino_GFX setting
 * 2. Upload Motion JPEG file
 *   FFat/LittleFS:
 *     upload FFat (FatFS) data with ESP32 Sketch Data Upload:
 *     ESP32: https://github.com/lorol/arduino-esp32fs-plugin
 ******************************************************************************/
#define MJPEG_FILENAME "/fxTQ.mjpeg"
#define MJPEG_BUFFER_SIZE (624 * 180 * 2 / 10) // memory for a single JPEG frame

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

#include <FFat.h>
#include <LittleFS.h>
#include <SPIFFS.h>
#include <SD.h>
#include <SD_MMC.h>

#include "MjpegClass.h"
MjpegClass mjpeg;

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

/* variables */
int total_frames = 0;
unsigned long total_read_video = 0;
unsigned long total_decode_video = 0;
unsigned long total_show_video = 0;
unsigned long start_ms, curr_ms;

uint16_t *img[24];
int cur_img_idx = 0;

// pixel drawing callback
int jpegDrawCallback(JPEGDRAW *pDraw)
{
  // Serial.printf("Draw pos = %d,%d. size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  gfx->draw16bitRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}

void cpy32(uint32_t *src, uint32_t *dst)
{
  int i = 640 * 180 / 2;
  while (i--)
  {
    *dst++ = *src++;
  }
}

void setup()
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX Motion JPEG Image Viewer example");

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
  ledcAttachPin(GFX_BL, 1); // assign RGB led pins to channels
  ledcSetup(1, 12000, 8);   // 12 kHz PWM, 8-bit resolution
  ledcWrite(1, 63);
#endif

  attachInterrupt(BUTTON_PIN, ISR, FALLING);
  attachInterrupt(INTERRUPT_PIN, ISR, FALLING);

  gfx->setTextColor(GREEN);
  gfx->setFont(u8g2_font_logisoso92_tn);

  if (!FFat.begin())
  // if (!LittleFS.begin())
  // if (!SPIFFS.begin())
  // if (!SD.begin(SS))
  // pinMode(10 /* CS */, OUTPUT);
  // digitalWrite(10 /* CS */, HIGH);
  // SD_MMC.setPins(12 /* CLK */, 11 /* CMD/MOSI */, 13 /* D0/MISO */);
  // if (!SD_MMC.begin("/root", true))
  {
    Serial.println(F("ERROR: File System Mount Failed!"));
    gfx->println(F("ERROR: File System Mount Failed!"));
  }
  else
  {
    File mjpegFile = FFat.open(MJPEG_FILENAME, "r");
    // File mjpegFile = LittleFS.open(MJPEG_FILENAME, "r");
    // File mjpegFile = SPIFFS.open(MJPEG_FILENAME, "r");
    // File mjpegFile = SD.open(MJPEG_FILENAME, "r");
    // File mjpegFile = SD_MMC.open(MJPEG_FILENAME, "r");

    if (!mjpegFile || mjpegFile.isDirectory())
    {
      Serial.println(F("ERROR: Failed to open " MJPEG_FILENAME " file for reading"));
      gfx->println(F("ERROR: Failed to open " MJPEG_FILENAME " file for reading"));
    }
    else
    {
      uint8_t *mjpeg_buf = (uint8_t *)malloc(MJPEG_BUFFER_SIZE);
      if (!mjpeg_buf)
      {
        Serial.println(F("mjpeg_buf malloc failed!"));
      }
      else
      {
        Serial.println(F("MJPEG start"));

        start_ms = millis();
        curr_ms = millis();
        mjpeg.setup(
            &mjpegFile, mjpeg_buf, jpegDrawCallback, false /* useBigEndian */,
            16 /* x */, 0 /* y */, gfx->width() /* widthLimit */, gfx->height() /* heightLimit */);

        while (mjpegFile.available() && mjpeg.readMjpegBuf())
        {
          // Read video
          total_read_video += millis() - curr_ms;
          curr_ms = millis();

          // Play video
          mjpeg.drawJpg();
          total_decode_video += millis() - curr_ms;

          curr_ms = millis();
          gfx->flush();
          total_show_video += millis() - curr_ms;

          img[cur_img_idx] = (uint16_t *)aligned_alloc(16, 640 * 180 * 2);
          cpy32((uint32_t *)gfx->getFramebuffer(), (uint32_t *)img[cur_img_idx++]);

          curr_ms = millis();
          total_frames++;
        }
        int time_used = millis() - start_ms;
        Serial.println(F("MJPEG end"));
        mjpegFile.close();
        float fps = 1000.0 * total_frames / time_used;
        // total_decode_video -= total_show_video;
        Serial.printf("Total frames: %d\n", total_frames);
        Serial.printf("Time used: %d ms\n", time_used);
        Serial.printf("Average FPS: %0.1f\n", fps);
        Serial.printf("Read MJPEG: %lu ms (%0.1f %%)\n", total_read_video, 100.0 * total_read_video / time_used);
        Serial.printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video, 100.0 * total_decode_video / time_used);
        Serial.printf("Show video: %lu ms (%0.1f %%)\n", total_show_video, 100.0 * total_show_video / time_used);

        // gfx->setCursor(0, 0);
        // gfx->printf("Total frames: %d\n", total_frames);
        // gfx->printf("Time used: %d ms\n", time_used);
        // gfx->printf("Average FPS: %0.1f\n", fps);
        // gfx->printf("Read MJPEG: %lu ms (%0.1f %%)\n", total_read_video, 100.0 * total_read_video / time_used);
        // gfx->printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video, 100.0 * total_decode_video / time_used);
        // gfx->printf("Show video: %lu ms (%0.1f %%)\n", total_show_video, 100.0 * total_show_video / time_used);
      }
    }
  }

  gfx->setCursor(36, 132);
  gfx->print("00:00:00.00");
  gfx->flush();
}

void loop()
{
  if (timing)
  {
    if (cur_img_idx >= 24)
    {
      cur_img_idx = 0;
    }

    cpy32((uint32_t *)img[cur_img_idx++], (uint32_t *)gfx->getFramebuffer());

    unsigned long time = millis() - start_time;

    time /= 10;
    int ss = time % 100;
    time /= 100;
    int sec = time % 60;
    time /= 60;
    int min = time % 60;
    time /= 60;
    sprintf(buf, "%.2d:%.2d:%.2d.%.2d", time, min, sec, ss);

    gfx->setCursor(36, 132);
    gfx->print(buf);
    gfx->flush();
  }
  else
  {
    delay(1);
  }
}
