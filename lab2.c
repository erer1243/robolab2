#include "kipr/wombat.h"
#include <math.h>
#include <stdbool.h>

// Movement helpers
typedef struct {
  float lspeed, rspeed; // -100 - 100
} Direction;

const Direction forward = {.lspeed = 80, .rspeed = 80};
const Direction backward = {.lspeed = -80, .rspeed = -80};
const Direction left = {.lspeed = -80, .rspeed = 80};
const Direction right = {.lspeed = 80, .rspeed = -80};

// move motors in Direction d
void go(Direction d) {
  mav(0, (int)(fmin(100, fmax(-100, d.lspeed)) * -15)); // left
  mav(1, (int)(fmin(100, fmax(-100, d.rspeed)) * 15));  // right
}

// go() for ms milliseconds
void go_for(Direction d, unsigned int ms) {
  go(d);
  msleep(ms);
  alloff();
}

// Bumper helpers
typedef struct {
  bool left, right;
} BumperState;

BumperState bumper(void) {
  return (BumperState){.left = digital(0), .right = digital(1)};
}

// Check bumper state and avoid the obstacle if one is bumped
void check_and_respond_to_bumpers(void) {
  BumperState bs = bumper();

  alloff();
  if (bs.left) {
    go_for(backward, 500);
    go_for(right, 300);
  } else if (bs.right) {
    go_for(backward, 500);
    go_for(left, 300);
  }
}

// Bump sensor lab task driver
void bump_sensor_main(void) {
  while (true) {
    go(forward);
    msleep(50);
    check_and_respond_to_bumpers();
  }
}

// Light sensor helpers
typedef struct {
  int left, right; // 4095 - 0
} LightSensorState;

LightSensorState light(void) {
  return (LightSensorState){
      .left = analog(0),
      .right = analog(1),
  };
}

// Normalize a light reading of 4095 - 0 to a motor speed value of -100 - 100
float normalize(int l) {
  const int saturate_dark = 3300;
  const int saturate_light = 2000;
  // Create a linear equation, y = mx + b, from saturate_light and saturate_dark
  // where x = light sensor value, y = normalized motor speed
  const float m = -100.0 / (saturate_dark - saturate_light);
  const float b = -saturate_dark * m;

  // Above or below saturating values, don't move or move at full speed
  if (l > saturate_dark)
    return 0;
  else if (l < saturate_light)
    return 100;
  // Between saturating values, use linear equation to normalize light readings
  else
    return m * l + b;
}

// Light following lab task driver
void light_sensor_main(void) {
  while (true) {
    LightSensorState lss = light();
    float norm_left = normalize(lss.left), norm_right = normalize(lss.right);
    Direction d = {
        .lspeed = norm_right - 0.1 * norm_left,
        .rspeed = norm_left - 0.1 * norm_right,
    };
    go_for(d, 50);
    check_and_respond_to_bumpers();
  }
}

// IR reflectance / line following helpers
typedef struct {
  int left, right; // 4096 - 0
} IRSensorState;

IRSensorState ir(void) {
  return (IRSensorState){
      .left = analog(2),
      .right = analog(3),
  };
}

const Direction forward_slow = {.lspeed = 30, .rspeed = 30};
const Direction right_slow = {.lspeed = 30, .rspeed = -30};
const Direction left_slow = {.lspeed = -30, .rspeed = 30};

// Check if either ir sensor senses the line, and turn towards the line if so.
void check_and_respond_to_ir(void) {
  const int threshold = 1200;
  const int turn_time = 300;
  IRSensorState iss = ir();

  if (iss.left > threshold)
    go_for(left_slow, turn_time);
  else if (iss.right > threshold)
    go_for(right_slow, turn_time);
}

// Line following lab task driver
void line_follow_main(void) {
  while (true) {
    go_for(forward_slow, 50);
    check_and_respond_to_ir();
  }
}

int main(void) {
  // bump_sensor_main();
  // light_sensor_main();
  line_follow_main();

  return 0;
}
