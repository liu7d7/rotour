// hi brandon
// https://abyz.me.uk/rpi/pigpio/cif.html
#include <pigpio.h>
#include <tgmath.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include "parser.c"
#include <stdbool.h>

#define c0_in1 22
#define c0_in2 10
#define c0_in3 9
#define c0_in4 27

#define c1_in1 24
#define c1_in2 25
#define c1_in3 20
#define c1_in4 21

#define button_pin 26

#define mc_thresh 0.35

#define err(...) do { \
  fprintf(stderr, "%s:%s:%d > ", __FILE__, __func__, __LINE__); \
  fprintf(stderr, __VA_ARGS__); \
  exit(-1); } while (false);

long long start_time;

float
time_s(bool reset)
{
  struct timeval tv;
  if (gettimeofday(&tv, NULL) == 0) {
    long long us = (long long)tv.tv_sec * 1000000 + tv.tv_usec;
    if (reset) start_time = us;
    return (float)(us - start_time) / 1000000.f;
  } 

  err("failed to get time!");
}

typedef struct a4990 {
  int in1, in2, in3, in4;
  float dir1, dir2;
} a4990;

#define pwm_range 20000

a4990
a4990_new(int in1, int in2, int in3, int in4, float dir1, float dir2)
{
  gpioSetMode(in1, PI_OUTPUT);
  gpioSetMode(in2, PI_OUTPUT);
  gpioSetMode(in3, PI_OUTPUT);
  gpioSetMode(in4, PI_OUTPUT);

  gpioPWM(in1, 0);
  gpioPWM(in2, 0);
  gpioPWM(in3, 0);
  gpioPWM(in4, 0);

  gpioSetPWMfrequency(in1, 100000);
  gpioSetPWMfrequency(in2, 100000);
  gpioSetPWMfrequency(in3, 100000);
  gpioSetPWMfrequency(in4, 100000);

  gpioSetPWMrange(in1, pwm_range);
  gpioSetPWMrange(in2, pwm_range);
  gpioSetPWMrange(in3, pwm_range);
  gpioSetPWMrange(in4, pwm_range);

  return (a4990){in1, in2, in3, in4, dir1, dir2};
}

void
a4990_set_pwr(a4990 *this, float pw1, float pw2) 
{
  pw1 = fmin(1.0, fmax(-1.0, pw1)) * this->dir1;
  pw2 = fmin(1.0, fmax(-1.0, pw2)) * this->dir2;

  if (pw1 > 0) {
    gpioPWM(this->in1, (unsigned)(fabs(pw1) * pwm_range));
    gpioPWM(this->in2, (unsigned)(0));
  } else {
    gpioPWM(this->in2, (unsigned)(fabs(pw1) * pwm_range));
    gpioPWM(this->in1, (unsigned)(0));
  }

  if (pw2 > 0) {
    gpioPWM(this->in3, (unsigned)(fabs(pw2) * pwm_range));
    gpioPWM(this->in4, (unsigned)(0));
  } else {
    gpioPWM(this->in4, (unsigned)(fabs(pw2) * pwm_range));
    gpioPWM(this->in3, (unsigned)(0));
  }
}

typedef struct pinpoint {
  float x, y, h;
  unsigned x_enc, y_enc;
  int handle;
} pinpoint;

enum pinpoint_reg {
  ppr_dev_id = 1,
  ppr_dev_ver,
  ppr_dev_status,
  ppr_dev_ctrl,
  ppr_loop_time,
  ppr_x_enc_val,
  ppr_y_enc_val,
  ppr_x,
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

typedef union pinpoint_val {
  int i;
  float f;
} pinpoint_val;

void
pinpoint_set_x_pod_offset(pinpoint *this, float x_pod_offset)
{
  i2cWriteI2CBlockData(this->handle, ppr_x_pod_offset, (char *)&x_pod_offset, 4);
}

void
pinpoint_set_y_pod_offset(pinpoint *this, float y_pod_offset)
{
  i2cWriteI2CBlockData(this->handle, ppr_y_pod_offset, (char *)&y_pod_offset, 4);
}

void
pinpoint_initialize(pinpoint *this)
{
  i2cWriteI2CBlockData(this->handle, ppr_dev_ctrl, (char *)&(int){2}, 4);
}

void
pinpoint_set_ticks_per_mm(pinpoint *this, float res)
{
  i2cWriteI2CBlockData(this->handle, ppr_ticks_per_mm, (char *)&res, 4);
}

pinpoint
pinpoint_new(int bus, int addr, int x_pod_offset, int y_pod_offset)
{
  int handle = i2cOpen(bus, addr, 0);

  if (handle < 0) err("failed to open pinpoint!");

  pinpoint out = (pinpoint){.handle = handle};

  pinpoint_set_x_pod_offset(&out, x_pod_offset);
  pinpoint_set_y_pod_offset(&out, y_pod_offset);
  pinpoint_set_ticks_per_mm(&out, 13.26291192f);

  pinpoint_initialize(&out);

  return out;
}

float
angle_wrap(float x)
{
  static const float pi = 3.1415926f;

  x = fmod(x + pi, 2 * pi);
  if (x < 0) x += 2 * pi;
  return x - pi;
}

void
pinpoint_update(pinpoint *this) 
{
  static pinpoint_val buf[8];

  i2cReadI2CBlockData(this->handle, ppr_bulk_read, (char *)buf, 32);
  this->x = buf[4].f * 1e-3, this->y = buf[5].f * 1e-3, this->h = angle_wrap(buf[6].f);
  this->x_enc = buf[2].i, this->y_enc = buf[3].i;
}

void
pinpoint_set_pos(pinpoint *this, float x, float y, float h) 
{
  i2cWriteI2CBlockData(this->handle, ppr_x, (char *)&x, 4);
  i2cWriteI2CBlockData(this->handle, ppr_y, (char *)&y, 4);
  i2cWriteI2CBlockData(this->handle, ppr_h, (char *)&h, 4);
}

typedef struct pidf {
  float p, i, d, f;
  float last_err, err_sum, last_time, goal;
  bool first_run;
} pidf;

pidf
pidf_new(float p, float i, float d, float f)
{
  return (pidf){p, i, d, f, .first_run = true};
}

float cur_time;

float
pidf_calc(pidf *this, float error, bool sq) 
{
  float p, i = 0, d = 0;

  if (sq) {
    p = this->p * sqrt(fabs(error)) * copysign(1, error);
  } else {
    p = this->p * error;
  }
  
  if (!this->first_run) {
    if (this->d != 0) {
      d = this->d * (error - this->last_err) / (cur_time - this->last_time);
    }

    i = this->i * this->err_sum;
  }

  float f = this->f * copysign(1, error);

  this->first_run = false;
  this->err_sum += error * (cur_time - this->last_time);
  this->last_err = error;
  this->last_time = cur_time;

  return p + i + d + f;
}

a4990 mc_y, mc_x;
pinpoint pp;

// :zany_face:
#define ever (;;)

float 
motor_scale(float power) 
{
  return copysign(mc_thresh, power) + power * (1 - mc_thresh);
}

float
lerp(float a, float b, float delta)
{
  return a + (b - a) * delta;
}

void
get_interpolated_point(float max_time, float time, bool *over, Point *out)
{
  int n_segments = point_count - 1;
  float delta = time / max_time;

  if (delta >= 1.) {
    *out = points[point_count - 1];
    *over = true;
    return;
  }

  int this_seg = delta * n_segments;
  float seg_delta = delta * n_segments - floor(delta * n_segments);

  printf("\n\nvvvv\ndelta: %f, this_seg: %d, seg_delta: %f\n", delta, this_seg, seg_delta);

  *out = (Point){
    .x = lerp(points[this_seg].x, points[this_seg + 1].x, seg_delta),
    .y = lerp(points[this_seg].y, points[this_seg + 1].y, seg_delta)
  };

  *over = false;
}

#define happy_time 0.2

int button_ticks = 0;

void
update_button(void)
{
  if (gpioRead(button_pin)) {
    button_ticks++;
  } else {
    button_ticks = 0;
  }
}

void
one_run(void)
{
  FILE *log = fopen("log.txt", "w");

  pp = pinpoint_new(1, 0x31, 40., -40.);
  sleep(4);
  pinpoint_update(&pp);

  read_points();
  fflush(stdout);

  pidf ph = pidf_new(1.0, 0, 0.22, 0);
  pidf px = pidf_new(3.6, 0, 0.22, 0);
  pidf py = pidf_new(3.6, 0, 0.22, 0);

  time_s(1);

  bool has_reached_endpoint = false;
  float time_reached_endpoint = 0;

  Point endpoint = points[point_count - 1];

  for ever {
    if (button_ticks > 5) goto end;

    update_button();
    cur_time = time_s(0);
    pinpoint_update(&pp);

    Point target;
    bool over;
    get_interpolated_point(max_time - happy_time, cur_time, &over, &target);
 
    Point cur = {.x = pp.x, .y = pp.y};
    if (dist(cur, endpoint) < 0.02 && over) {
      if (!has_reached_endpoint) {
        has_reached_endpoint = true;
        time_reached_endpoint = cur_time;
      } else if (cur_time - time_reached_endpoint > happy_time) {
        goto end;
      }
    } else {
      has_reached_endpoint = false;
    }

    float erx = target.x - cur.x, ery = target.y - cur.y;
    float angle_to_target = atan2(ery, erx);
    float angle_error = angle_wrap(angle_to_target);

    float rotated_erx = erx * cos(-pp.h) - ery * sin(-pp.h);
    float rotated_ery = ery * cos(-pp.h) + erx * sin(-pp.h);

    float xc = pidf_calc(&px, rotated_erx, true);
    float yc = pidf_calc(&py, rotated_ery, true);
    float hc = pidf_calc(&ph, angle_wrap(pp.h), true);

    float a = yc - hc, b = yc + hc, c = xc + hc, d = xc - hc;
    float mm = fmax(fabs(a), fmax(fabs(b), fmax(fabs(c), fabs(d))));
    if (mm > 1) {
      a /= mm, b /= mm, c /= mm, d /= mm;
    }

    a4990_set_pwr(&mc_y, motor_scale(a), motor_scale(b));
    a4990_set_pwr(&mc_x, motor_scale(c), motor_scale(d));

    printf("x: %.3f, y: %.3f, h: %.3f, tx: %f, ty: %f, time: %f, xc: %f, yc: %f, hc: %f\n", pp.x, pp.y, pp.h, target.x, target.y, cur_time, xc, yc, hc);
    fprintf(log, "x: %.3f, y: %.3f, h: %.3f, tx: %.3f, ty: %.3f\n", pp.x, pp.y, pp.h, target.x, target.y);
  }

end:

  button_ticks = 0;

  a4990_set_pwr(&mc_x, 0, 0);
  a4990_set_pwr(&mc_y, 0, 0);

  fclose(log);
}

int
main(void)
{
  gpioInitialise(); // if this fails it's cooked anyways so why handle the error
  gpioCfgClock(2, 0, 0);

  gpioSetMode(button_pin, PI_INPUT);

  mc_y = a4990_new(c0_in1, c0_in2, c0_in3, c0_in4, 1, 1);
  mc_x = a4990_new(c1_in1, c1_in2, c1_in3, c1_in4, 1, -1);

  a4990_set_pwr(&mc_x, 0, 0);
  a4990_set_pwr(&mc_y, 0, 0);

  for ever {
    while (button_ticks < 5) {
      update_button();

      usleep(100000);
    }

    sleep(2);

    button_ticks = 0;

    one_run();
  }

  gpioTerminate();
}
