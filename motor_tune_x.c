#include <stdio.h>
#include <unistd.h>
#include <pigpio.h>
#include <math.h>

typedef struct a4990 {
  int in1, in2, in3, in4;
  float dir1, dir2;
} a4990;

#define pwm_range 20000

#define mc_thresh 0.35

#define err(...) do { \
  fprintf(stderr, "%s:%s:%d > ", __FILE__, __func__, __LINE__); \
  fprintf(stderr, __VA_ARGS__); \
  exit(-1); } while (false);


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

  fprintf(stderr, "%d", gpioGetPWMrange(in1));

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

#define c1_in1 24
#define c1_in2 25
#define c1_in3 20
#define c1_in4 21

a4990 mc_x;

float 
motor_scale(float power) 
{
  return copysign(mc_thresh, power) + power * (1 - mc_thresh);
}

#define button_pin 26

int
main() {
  gpioInitialise(); 

  gpioSetMode(button_pin, PI_INPUT);

  mc_x = a4990_new(c1_in1, c1_in2, c1_in3, c1_in4, 1, -1);

  for (int i = 0; i <= 10; i++) {
    if (gpioRead(button_pin)) break;
    a4990_set_pwr(&mc_x, motor_scale((float)i * 0.1), motor_scale((float)i * 0.1));
    printf("%d\n", i);
    sleep(5);
  }

  a4990_set_pwr(&mc_x, 0, 0);
}
