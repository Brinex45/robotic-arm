#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver PCA = Adafruit_PWMServoDriver(0x40, Wire);

bool code = 0;

byte psData[5];
byte butt;
bool L1 = 0, R1 = 0;
bool reset = 0;
bool gripper = 0;
float leftX = 0, leftY = 0, rightY = 0, oMega = 0;
int8_t X_left = 0, Y_left = 0, Y_right = 0, oMega_R = 0;

int theta_1 = 0, theta_2 = 0, theta_3 = 0, theta_4 = -90, theta_5 = 0;
int theta_1_ang = 0, theta_2_ang = 0, theta_3_ang = 0, theta_4_ang = 0, theta_5_ang = 0;

bool gripper_flag = 0;
int gripper_close = 0, gripper_open = 90;
int gripper_ang;
int grip_ang;

void Write(bool is180, int ServoNumber, int temp_pos) {
  int pulseLength;
  const int ServoMin180 = 102;
  const int ServoMax180 = 495;
  int pos = constrain(temp_pos, 0, (is180 ? 180 : 270));

  if (is180) {
    pulseLength = map(pos, 0, 180, ServoMin180, ServoMax180);
  } else {
    pulseLength = map(pos, 0, 270, 102, 512);
  }

  PCA.setPWM(ServoNumber, 0, pulseLength);
}

void get_data() {
  Wire2.requestFrom(8, 5);
  while (Wire2.available() == 5) {

    Wire2.readBytes(psData, 5);
    butt = psData[0];
    X_left = psData[1];
    Y_left = psData[2];
    Y_right = psData[3];
    oMega_R = psData[4];
    if (oMega_R == -128) {
      oMega_R = 127;
    }
    // Serial.println((String)X_left + "    " + Y_left + "   " + Y_right + "    " + oMega_R + "   " + butt);
  }


  leftX = map(float(X_left), float(-127), float(127), float(-1), float(1));
  leftY = map(float(Y_left), float(-127), float(127), float(-1), float(1));
  rightY = map(float(Y_right), float(-127), float(127), float(-1), float(1));
  oMega = map(float(oMega_R), float(-127), float(127), float(-1), float(1));

  leftX = abs(leftX) > 0.15 ? leftX : 0;
  leftY = abs(leftY) > 0.15 ? leftY : 0;
  rightY = abs(rightY) > 0.15 ? rightY : 0;
  oMega = abs(oMega) > 0.15 ? oMega : 0;

  L1 = butt & 1 << 0;
  R1 = butt & 1 << 1;
  reset = butt & 1 << 6;
  gripper = butt & 1 << 7;

  if (gripper) {
    gripper_flag = gripper_flag ? 0 : 1;
    gripper_ang = gripper_flag ? gripper_open : gripper_close;
    // Serial.println("aaa");
  }
  // Serial.print((String)leftX + "    " + leftY + "   " + rightY + "    " + oMega + "   " + butt);
}

void angle_adjust() {
  theta_1_ang = constrain(map(theta_1, -90, 90, 180, 0), 0, 180);
  theta_2_ang = constrain(map(theta_2, 0, -180, 0, 180), 0, 180);
  theta_3_ang = constrain(map(theta_3, -90, 90, 180, 0), 0, 180);
  theta_4_ang = constrain(map(theta_4, -90, 90, 180, 0), 0, 180);
  theta_5_ang = constrain(map(theta_5, -90, 90, 180, 0), 0, 180);
  grip_ang = constrain(map(gripper_ang, -90, 90, 180, 0), 0, 180);
}

void move_servos() {

  if (abs(leftX))
    theta_1 = constrain(theta_1 + (leftX * 2), -90, 90);

  if (abs(leftY))
    theta_2 = constrain(theta_2 + (leftY * 2), -90, 90);

  if (abs(rightY))
    theta_3 = constrain(theta_3 + (rightY * 2), -90, 90);

  if (abs(oMega))
    theta_4 = constrain(theta_4 + (oMega * 2), -90, 90);

  if (L1 || R1)
    theta_5 = constrain(theta_5 + (R1 - L1), -90, 90);
  

  // if (abs(leftX) || abs(leftY) || abs(rightY) || abs(oMega) || L1 || R1) {
    angle_adjust();
  // }

  Serial.print((String) "    " + theta_1_ang + "    " + theta_2_ang + "   " + theta_3_ang + "   " + theta_4_ang + "   " + theta_5_ang + "   " + grip_ang);
  Serial.println((String) "    " + theta_1 + "    " + theta_2 + "   " + theta_3 + "   " + theta_4 + "   " + theta_5 + "   " + grip_ang);
  Write(true, 8, theta_1_ang);
  Write(true, 12, theta_2_ang);
  Write(true, 15, theta_3_ang);
  Write(true, 11, theta_4_ang);
  Write(true, 10, theta_5_ang);
  Write(true, 6, grip_ang);

  // if (gripper_flag)
  //   Write(true, 6, gripper_open);
  // else
  //   Write(true, 6, gripper_close);
}

void servo_zero() {
  Write(true, 8, 0);
  Write(true, 12, 0);
  Write(true, 15, 0);
  Write(true, 11, 0);
  Write(true, 10, 0);
  // Write(true, 6, 0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Wire.begin();
  Wire2.begin();

  PCA.begin();
  PCA.setPWMFreq(50);

  digitalWrite(13, HIGH);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  // if(code)
  //   servo_zero();
  // else
  get_data();
  move_servos();
  delay(15);
}
