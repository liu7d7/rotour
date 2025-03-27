// hi brandon
// https://abyz.me.uk/rpi/pigpio/cif.html
#include <pigpio.h>
#include <tgmath.h>
#include <unistd.h>

#define c0_in1 4
#define c0_in2 17
#define c0_in3 27
#define c0_in4 22

#define c1_in1 14
#define c1_in2 15
#define c1_in3 18
#define c1_in4 23

#define button_pin 24

#define err(...) \
  fprintf(stderr, "%s:%s:%d > ", __FILE__, __func__, __LINE__); \
  fprintf(stderr, __VA_ARGS__);

float clamp()
{

}

typedef struct a4990 {
  int in1, in2, in3, in4;
} a4990;

a4990
a4990_new(int in1, int in2, int in3, int in4)
{
  gpioPWM(in1, 0);
  gpioPWM(in2, 0);
  gpioPWM(in3, 0);
  gpioPWM(in4, 0);

  gpioSetPWMFrequency(in1, 21700);
  gpioSetPWMFrequency(in2, 21700);
  gpioSetPWMFrequency(in3, 21700);
  gpioSetPWMFrequency(in4, 21700);

  return (a4990){in1, in2, in3, in4};
}

void
a4990_set_pwr(a4990 *this, float pw1, float pw2) 
{
  pw1 = min(1.0, max(0.0, pw1));
  pw2 = min(1.0, max(0.0, pw2));

  gpioPWM(this->in1, (unsigned)(pw1 * 255));
  gpioPWM(this->in2, (unsigned)(255 - pw1 * 255));

  gpioPWM(this->in3, (unsigned)(pw2 * 255));
  gpioPWM(this->in4, (unsigned)(255 - pw2 * 255));
}

typedef struct pinpoint {
  float x, y, h; 
  int handle;
} pinpoint;

enum pinpoint_registers {
  ppr_dev_id = 1,
  ppr_dev_ver,
  ppr_dev_status,
  ppr_dev_ctrl,
  ppr_loop_time,
  ppr_x_enc_val,
  ppr_x
  ppr_y,
  ppr_h,
  ppr_vx,
  ppr_vy,
  ppr_vh,
  ppr_ticks_per_mm,
  ppr_x_pod_offset,
  ppr_y_pod_offset,
  ppr_yaw_offset,
  ppr_bulk_read
};

void
pinpoint_set_x_pod_offset(pinpoint *this, float x_pod_offset)
{
  char buf[4];
  *(float *)buf = x_pod_offset;

  i2cWriteBlockData(this->handle, ppr_x_pod_offset, buf, 4);
}

void
pinpoint_set_y_offset(pinpoint *this, float y_pod_offset)
{
  char buf[4];
  *(float *)buf = y_pod_offset;

  i2cWriteBlockData(this->handle, ppr_y_pod_offset, buf, 4);
}

pinpoint
pinpoint_new(int bus, int addr, int x_pod_offset, int y_pod_offset)
{
  int handle = i2cOpen(bus, addr, 0);

  if (handle < 0) err("failed to open pinpoint!");

  pinpoint out = (pinpoint){.handle = handle};
  pinpoint_set_x_pod_offset(&out, x_pod_offset);
  pinpoint_set_y_pod_offset(&out, y_pod_offset);

  return out;
}

void
pinpoint_update(pinpoint *this) 
{
  i2cReadI2CBlockData(this->handle, ppr_x, (char *)&this->x, 4);
  i2cReadI2CBlockData(this->handle, ppr_y, (char *)&this->y, 4);
  i2cReadI2CBlockData(this->handle, ppr_h, (char *)&this->h, 4);
}

void
pinpoint_set_pos(pinpoint *this, float x, float y, float h) 
{
  i2cWriteI2CBlockData(this->handle, ppr_x, (char *)&x, 4);
  i2cWriteI2CBlockData(this->handle, ppr_y, (char *)&y, 4);
  i2cWriteI2CBlockData(this->handle, ppr_h, (char *)&h, 4);
}

a4990 mc1, mc2;
pinpoint pp;

// :zany_face:
#define ever (;;)

int
main(void)
{
  gpioInitialise(); // if this fails it's cooked anyways so why handle the error
  
  mc1 = a4990_new(c0_in1, c0_in2, c0_in3, c0_in4);
  mc2 = a4990_new(c1_in1, c1_in2, c1_in3, c1_in4);

  pp = pinpont_new(0, 0x31);

  gpioSetMode(button_pin, PI_INPUT);

  for ever {
    // wait until button press

    int times = 0;
    for ever {
      if (gpioRead(button_pin)) times++;
      else times = 0;

      if (times == 5)
    }
  }
}
