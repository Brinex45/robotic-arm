#include <Servo.h>

#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include <Wire.h>
#include <Encoder.h>

// #define Rpwm_base 1
// #define Lpwm_base 1

Servo s;

#define kd 10
#define ki 0
#define kp 0.7


#define Rpwm_shoulder 2
#define Lpwm_shoulder 3

#define Rpwm_elbow 6
#define Lpwm_elbow 7

#define Rpwm_wrist 9
#define Lpwm_wrist 8

// #define Rpwm_wrist_rot 8
// #define Lpwm_wrist_rot 9

// Encoder enco_base(0, 0);
Encoder enco_shoulder(30, 31);
Encoder enco_elbow(38, 37);
Encoder enco_wrist(26, 27);
// Encoder enco_wrist_rot(32, 33);

#define cpr_base 0
#define cpr_shoulder 8192
#define cpr_elbow 8192
#define cpr_wrist 8192
#define cpr_wrist_rot 0

BLA::Matrix<4, 4> home_matrix = {
  0, 0, 1, 531.3,
  0, 1, 0, 0,
  -1, 0, 0, 230.5,
  0, 0, 0, 1
};

BLA::Matrix<4, 4> Transform_matrix = home_matrix;

#define unit_trans 1
#define unit_rot 0.5

#define d1 37.5
#define d5 80.5
#define a2 450
#define a3 450

byte psData[5] = { 0 };
byte buttons;
int i = 80;
int8_t X_left;
int8_t Y_left;
int8_t Y_right;
int8_t oMega;
bool L1, R1;
bool Ps, Touch, Share, Option;

float X_trans, Y_trans, Z_trans, Y_rot, Z_rot;
float X_prev, Y_prev, Z_prev;

float theta_1, theta_2, theta_3, theta_4, theta_5;
float theta_234;

float X_left_m, Y_left_m, Y_right_m, oMega_m;

void get_data() {
  Wire1.requestFrom(8, 5);
  while (Wire1.available() == 5) {

    Wire1.readBytes(psData, 5);
    buttons = psData[0];
    X_left = psData[1];
    Y_left = psData[2];
    Y_right = psData[3];
    oMega = psData[4];
    if (oMega == -128) {
      oMega = 127;
    }
    // Serial.println((String)buttons + "    " + X_left + "    " + Y_left + "    " + Y_right + "   " + oMega);
  }

  if (abs(X_left) <= 20) {
    X_left = 0;
  }
  if (abs(Y_left) <= 20) {
    Y_left = 0;
  }
  if (abs(Y_right) <= 20) {
    Y_right = 0;
  }
  if (abs(oMega) <= 20) {
    oMega = 0;
  }

  L1 = buttons & 1 << 0;
  R1 = buttons & 1 << 1;
  Share = buttons & 1 << 4;
  Option = buttons & 1 << 5;
  Touch = buttons & 1 << 6;
  Ps = buttons & 1 << 7;

  mapping();
}

void mapping() {
  X_left_m = map(float(X_left), -127, 127, -1.0, 1.0);
  Y_left_m = map(float(Y_left), -127, 127, -1.0, 1.0);
  Y_right_m = map(float(Y_right), -127, 127, -1.0, 1.0);
  oMega_m = map(float(oMega), -127, 127, -1.0, 1.0);

  // Serial.println((String)buttons + "    " + X_left_m + "    " + Y_left_m + "    " + Y_right_m + "   " + oMega_m);

  if (X_left_m) {
    X_trans += (X_left_m * unit_trans);
  }
  if (Y_left_m) {
    Y_trans += (Y_left_m * unit_trans);
  }
  if (Y_right_m) {
    Z_trans += (Y_right_m * unit_trans);
  }
  if (oMega_m) {
    Y_rot = (oMega_m * unit_rot);
  } else Y_rot = 0;
  if (L1 || R1) {
    Z_rot = (L1 - R1) * unit_rot;
  } else Z_rot = 0;
  // Serial.println(X_trans);
}

void matrix_prep() {
  Y_rot = DEG_TO_RAD * Y_rot;
  Z_rot = DEG_TO_RAD * Z_rot;

  BLA::Matrix<4, 4> Y_rot_new = {
    cos(Y_rot), 0, sin(Y_rot), 0,
    0, 1, 0, 0,
    -sin(Y_rot), 0, cos(Y_rot), 0,
    0, 0, 0, 1
  };

  BLA::Matrix<4, 4> Z_rot_new = {
    cos(Z_rot), -sin(Z_rot), 0, 0,
    sin(Z_rot), cos(Z_rot), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };

  BLA::Matrix<4, 4> Trans = {
    0, 0, 0, X_trans - X_prev,
    0, 0, 0, Y_trans - Y_prev,
    0, 0, 0, Z_trans - Z_prev,
    0, 0, 0, 1
  };
  X_prev = X_trans;
  Y_prev = Y_trans;
  Z_prev = Z_trans;

  Transform_matrix = (Transform_matrix * Y_rot_new * Z_rot_new) + Trans;
}

void kinematics() {
  matrix_prep();

  float nx = Transform_matrix(0, 0);
  float ny = Transform_matrix(1, 0);
  float nz = Transform_matrix(2, 0);

  float sx = Transform_matrix(0, 1);
  float sy = Transform_matrix(1, 1);
  float sz = Transform_matrix(2, 1);

  float ax = Transform_matrix(0, 2);
  float ay = Transform_matrix(1, 2);
  float az = Transform_matrix(2, 2);

  float Px = Transform_matrix(0, 3);
  float Py = Transform_matrix(1, 3);
  float Pz = Transform_matrix(2, 3);

  theta_1 = atan2(Py, Px);
  theta_5 = atan2(sz, -nz);

  theta_234 = atan2(-((ax * cos(theta_1)) + (ay * sin(theta_1))), -az);

  float c = (Px / cos(theta_1)) + d5 * sin(theta_234);
  float d = d1 - d5 * cos(theta_234) - Pz;

  float costheta3 = (sq(c) + sq(d) - sq(a2) - sq(a3)) / (2 * a2 * a3);
  costheta3 = constrain(costheta3, -1, 1);
  float sintheta3 = sqrt(1 - sq(costheta3));

  theta_3 = atan2(sintheta3, costheta3);

  float r = a3 * cos(theta_3) + a2;
  float s = a3 * sin(theta_3);

  theta_2 = atan2(r * d - s * c, r * c + s * d);
  theta_4 = theta_234 - theta_2 - theta_3 + PI;

  // Serial.println((String)c + "    " + d + "   " + r + "   " + s + "   " + costheta3*RAD_TO_DEG + "    " + theta_234*RAD_TO_DEG);

  theta_1 = RAD_TO_DEG * theta_1;
  theta_2 = RAD_TO_DEG * theta_2;
  theta_3 = RAD_TO_DEG * theta_3;
  theta_4 = RAD_TO_DEG * theta_4;
  theta_5 = RAD_TO_DEG * theta_5;

  // Serial.println((String)theta_1 + "    " + theta_2 + "   " + theta_3 + "   " + theta_4 + "   " + theta_5);
}

class joints {
private:
  long cpr;
  int Rpwm;
  int Lpwm;
  int init_angle;
  Encoder& enco;
  int pwm;

  int tar_angle = 0;
  int cur_angle = 0;
  int reach_ang;

  float currT, prevT;
  float prevError, derivative_error, integral_error, error;
public:
  joints(int Rpwm, int Lpwm, long cpr, float init_angle, int pwm, Encoder& enco)
    : enco(enco) {
    this->Rpwm = Rpwm;
    this->Lpwm = Lpwm;
    this->pwm = pwm;
    this->cpr = cpr;
    this->init_angle = init_angle;
  }

  void pid(){
    error = reach_ang - tar_angle;

    currT = micros();
    derivative_error = (error - prevError) / (currT - prevT);
    integral_error += error;
    pwm = (kp * error + (derivative_error * kd) + (integral_error * ki));
    prevError = error;
    prevT = currT;
  }

  void motor(int r, int l) {
    analogWrite(Rpwm, r);
    analogWrite(Lpwm, l);
  }

  int t2a(int etick){
    // Serial.println(etick);
    return map(etick, 0, cpr, 0, 360);
    // return map(etick,0,cpr,0,360);
  }

  void acctuation(int tar_angle) {
    cur_angle = t2a(enco.read());

    reach_ang = cur_angle + init_angle;

    pid();

    if (tar_angle < reach_ang && abs(tar_angle - reach_ang) > 1) {
      motor(pwm, 0);
    } else if (tar_angle > reach_ang && abs(tar_angle - reach_ang) > 1) {
      motor(0, pwm);
    } else motor(0, 0);

    Serial.println((String)cur_angle + "    " + tar_angle + "   " + init_angle + "        " + reach_ang);
  }
};

// joints J_1(Rpwm_base, Lpwm_base, cpr_base, 0, 50, enco_base);
joints J_2(Rpwm_shoulder, Lpwm_shoulder, cpr_shoulder, -80, 255, enco_shoulder); // down inc, up dec
joints J_3(Rpwm_elbow, Lpwm_elbow, cpr_elbow, 113, 40, enco_elbow); // down inc, up dec
joints J_4(Rpwm_wrist, Lpwm_wrist, cpr_wrist, 56, 40, enco_wrist); // down dec, up inc
// joints J_5(Rpwm_wrist_rot, Lpwm_wrist_rot, cpr_wrist_rot, 0, 50, enco_wrist_rot);


void setup() {
  Serial.begin(115200);
  Wire1.begin();
  digitalWrite(13, HIGH);
  s.attach(34);
  Serial.println("aaa");

  int prev_sh = 0;
  int prev_el = 0;

  // while(1){
  //   J_2.motor(255, 0);

  //   delay(2000);
  //   if (J_2.t2a(enco_shoulder.read()) - prev_sh != 0) {
  //    break;
  //   }

  //   prev_sh = J_2.t2a(enco_shoulder.read());
  // }

  // while (1) {
  //   J_3.motor(120, 0);

  //   delay(500);
  //   if (J_3.t2a(enco_elbow.read()) - prev_el != 0) {
  //    break;
  //   }
  //   prev_el = J_3.t2a(enco_elbow.read());
  // }
}

void loop() {
  get_data();
  kinematics();
  // J_1.acctuation(theta_1);
  J_2.acctuation(theta_2);
  J_3.acctuation(theta_3);
  J_4.acctuation(theta_4);
  // J_5.acctuation(theta_5);

  if(Share){
    i += 2;
    Serial.println("a");
  } if(Option) {
    i -= 2;
    Serial.println("b");
  }
  i = constrain(i, 80, 180);
    s.write(i);

  Serial.println();
  delay(10);
}