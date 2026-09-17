/* CONTROLE V6 — ESP32-C3 + 2 joysticks + GY-521 + ESP-NOW */
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <esp_arduino_version.h>
#define JOY_L_X 0
#define JOY_L_Y 1
#define JOY_R_X 3
#define JOY_R_Y 4
#define BTN_MODE 5
#define MPU_SDA 6
#define MPU_SCL 7
#define BTN_ESTOP 10
#define BTN_SPEED 20
#define BTN_ACTION 21
const uint8_t BROADCAST_MAC[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const int JOY_DEADZONE=8; const float GESTURE_DEADZONE_DEG=7.0, GESTURE_MAX_DEG=32.0; const bool INVERT_LX=false,INVERT_LY=true,INVERT_RX=false,INVERT_RY=true; const int GESTURE_FORWARD_SIGN=-1,GESTURE_TURN_SIGN=+1;
static const uint32_t MAGIC_CONTROL=0x524F424F,MAGIC_TELEM=0x54454C45;
enum ControlMode:uint8_t{MODE_DUAL=0,MODE_TANK=1,MODE_ARCADE=2,MODE_GESTURE=3};
struct __attribute__((packed)) ControlPacket{uint32_t magic;uint16_t seq;uint8_t mode;int8_t leftMotor;int8_t rightMotor;uint8_t flags;};
struct __attribute__((packed)) TelemetryPacket{uint32_t magic;uint16_t lastSeq;float distanceCm;uint8_t obstacle;uint8_t failsafe;uint8_t emergency;};
ControlMode controlMode=MODE_DUAL; uint16_t sequenceNumber=0; bool emergency=false; int powerLevels[]={45,70,100},powerIndex=1; int centerLX=2048,centerLY=2048,centerRX=2048,centerRY=2048; bool lastModeBtn=HIGH,lastEstopBtn=HIGH,lastSpeedBtn=HIGH; unsigned long lastButtonMs=0,lastSendMs=0,lastSerialMs=0; volatile bool telemetryAvailable=false; TelemetryPacket latestTelemetry={};
const uint8_t MPU_ADDR=0x68; float gyroOffsetX=0,gyroOffsetY=0,gyroOffsetZ=0,pitchDeg=0,rollDeg=0,neutralPitch=0,neutralRoll=0; unsigned long lastImuUs=0;
bool mpuWrite(uint8_t reg,uint8_t value){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);Wire.write(value);return Wire.endTransmission()==0;}
bool mpuReadBytes(uint8_t reg,uint8_t*buffer,size_t len){Wire.beginTransmission(MPU_ADDR);Wire.write(reg);if(Wire.endTransmission(false)!=0)return false;size_t got=Wire.requestFrom((int)MPU_ADDR,(int)len,true);if(got!=len)return false;for(size_t i=0;i<len;i++)buffer[i]=Wire.read();return true;}
bool initMPU(){delay(50);if(!mpuWrite(0x6B,0x00))return false;delay(30);mpuWrite(0x1A,0x03);mpuWrite(0x1B,0x00);mpuWrite(0x1C,0x00);return true;}
bool readMPURaw(int16_t&ax,int16_t&ay,int16_t&az,int16_t&gx,int16_t&gy,int16_t&gz){uint8_t b[14];if(!mpuReadBytes(0x3B,b,sizeof(b)))return false;ax=(int16_t)((b[0]<<8)|b[1]);ay=(int16_t)((b[2]<<8)|b[3]);az=(int16_t)((b[4]<<8)|b[5]);gx=(int16_t)((b[8]<<8)|b[9]);gy=(int16_t)((b[10]<<8)|b[11]);gz=(int16_t)((b[12]<<8)|b[13]);return true;}
void calibrateMPU(){Serial.println("Calibrando GY-521...");const int samples=500;long sx=0,sy=0,sz=0;int valid=0;for(int i=0;i<samples;i++){int16_t ax,ay,az,gx,gy,gz;if(readMPURaw(ax,ay,az,gx,gy,gz)){sx+=gx;sy+=gy;sz+=gz;valid++;}delay(3);}if(valid>0){gyroOffsetX=(float)sx/valid;gyroOffsetY=(float)sy/valid;gyroOffsetZ=(float)sz/valid;}float pitchSum=0,rollSum=0;valid=0;for(int i=0;i<180;i++){int16_t ax,ay,az,gx,gy,gz;if(readMPURaw(ax,ay,az,gx,gy,gz)){float accPitch=atan2((float)ay,sqrt((float)ax*ax+(float)az*az))*180.0/PI;float accRoll=atan2(-(float)ax,(float)az)*180.0/PI;pitchSum+=accPitch;rollSum+=accRoll;valid++;}delay(4);}if(valid>0){neutralPitch=pitchSum/valid;neutralRoll=rollSum/valid;pitchDeg=neutralPitch;rollDeg=neutralRoll;}lastImuUs=micros();}
void updateMPU(){int16_t ax,ay,az,gx,gy,gz;if(!readMPURaw(ax,ay,az,gx,gy,gz))return;unsigned long nowUs=micros();float dt=(nowUs-lastImuUs)/1000000.0f;lastImuUs=nowUs;if(dt<=0||dt>0.2f)dt=0.01f;float accPitch=atan2((float)ay,sqrt((float)ax*ax+(float)az*az))*180.0f/PI;float accRoll=atan2(-(float)ax,(float)az)*180.0f/PI;float gyroX=((float)gx-gyroOffsetX)/131.0f,gyroY=((float)gy-gyroOffsetY)/131.0f;pitchDeg=0.98f*(pitchDeg+gyroX*dt)+0.02f*accPitch;rollDeg=0.98f*(rollDeg+gyroY*dt)+0.02f*accRoll;}
int normalizeAxis(int raw,int center,bool invertAxis){int value=0;if(raw>=center){int span=max(1,4095-center);value=(raw-center)*100/span;}else{int span=max(1,center);value=-((center-raw)*100/span);}value=constrain(value,-100,100);if(abs(value)<JOY_DEADZONE)value=0;if(invertAxis)value*=-1;return value;}
void calibrateJoysticks(){long sLX=0,sLY=0,sRX=0,sRY=0;const int samples=160;for(int i=0;i<samples;i++){sLX+=analogRead(JOY_L_X);sLY+=analogRead(JOY_L_Y);sRX+=analogRead(JOY_R_X);sRY+=analogRead(JOY_R_Y);delay(4);}centerLX=sLX/samples;centerLY=sLY/samples;centerRX=sRX/samples;centerRY=sRY/samples;}
int gestureValue(float angle,float neutral,int sign){float delta=(angle-neutral)*sign;if(fabs(delta)<GESTURE_DEADZONE_DEG)return 0;float magnitude=(fabs(delta)-GESTURE_DEADZONE_DEG)/(GESTURE_MAX_DEG-GESTURE_DEADZONE_DEG);magnitude=constrain(magnitude,0.0f,1.0f);int v=(int)(magnitude*100.0f);return delta>=0?v:-v;}
void mixArcade(int throttle,int steering,int&left,int&right){left=throttle+steering;right=throttle-steering;int peak=max(abs(left),abs(right));if(peak>100){left=left*100/peak;right=right*100/peak;}}
void applyPowerLimit(int&left,int&right,int maxPower){left=constrain(left*maxPower/100,-100,100);right=constrain(right*maxPower/100,-100,100);}
bool pressedEdge(int pin,bool&lastState){bool now=digitalRead(pin),pressed=(lastState==HIGH&&now==LOW);lastState=now;if(pressed&&millis()-lastButtonMs>140){lastButtonMs=millis();return true;}return false;}
const char*modeName(ControlMode mode){switch(mode){case MODE_DUAL:return"DUAL";case MODE_TANK:return"TANQUE";case MODE_ARCADE:return"ARCADE";case MODE_GESTURE:return"GESTOS";default:return"?";}}
void handleButtons(){if(pressedEdge(BTN_MODE,lastModeBtn))controlMode=(ControlMode)(((uint8_t)controlMode+1)%4);if(pressedEdge(BTN_ESTOP,lastEstopBtn))emergency=!emergency;if(pressedEdge(BTN_SPEED,lastSpeedBtn))powerIndex=(powerIndex+1)%3;}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t*info,const uint8_t*data,int len){
#else
void onDataRecv(const uint8_t*mac,const uint8_t*data,int len){
#endif
if(len!=sizeof(TelemetryPacket))return;TelemetryPacket packet;memcpy(&packet,data,sizeof(packet));if(packet.magic!=MAGIC_TELEM)return;latestTelemetry=packet;telemetryAvailable=true;}
bool initEspNow(){WiFi.mode(WIFI_STA);WiFi.disconnect();delay(100);if(esp_now_init()!=ESP_OK)return false;esp_now_register_recv_cb(onDataRecv);esp_now_peer_info_t peer={};memcpy(peer.peer_addr,BROADCAST_MAC,6);peer.channel=0;peer.encrypt=false;if(!esp_now_is_peer_exist(BROADCAST_MAC)&&esp_now_add_peer(&peer)!=ESP_OK)return false;return true;}
void setup(){Serial.begin(115200);delay(700);pinMode(BTN_MODE,INPUT_PULLUP);pinMode(BTN_ESTOP,INPUT_PULLUP);pinMode(BTN_SPEED,INPUT_PULLUP);pinMode(BTN_ACTION,INPUT_PULLUP);analogReadResolution(12);Wire.begin(MPU_SDA,MPU_SCL,400000);if(initMPU())calibrateMPU();calibrateJoysticks();if(!initEspNow())while(true)delay(1000);Serial.println("Controle pronto.");}
void loop(){updateMPU();handleButtons();const bool actionHeld=(digitalRead(BTN_ACTION)==LOW);int lx=normalizeAxis(analogRead(JOY_L_X),centerLX,INVERT_LX),ly=normalizeAxis(analogRead(JOY_L_Y),centerLY,INVERT_LY),rx=normalizeAxis(analogRead(JOY_R_X),centerRX,INVERT_RX),ry=normalizeAxis(analogRead(JOY_R_Y),centerRY,INVERT_RY),left=0,right=0;switch(controlMode){case MODE_DUAL:mixArcade(ly,rx,left,right);break;case MODE_TANK:left=ly;right=ry;break;case MODE_ARCADE:mixArcade(ly,lx,left,right);break;case MODE_GESTURE:if(actionHeld){int throttle=gestureValue(pitchDeg,neutralPitch,GESTURE_FORWARD_SIGN),steering=gestureValue(rollDeg,neutralRoll,GESTURE_TURN_SIGN);mixArcade(throttle,steering,left,right);}break;}int maxPower=powerLevels[powerIndex];if(controlMode!=MODE_GESTURE&&actionHeld)maxPower=100;applyPowerLimit(left,right,maxPower);if(emergency){left=0;right=0;}if(millis()-lastSendMs>=40){lastSendMs=millis();ControlPacket packet={};packet.magic=MAGIC_CONTROL;packet.seq=++sequenceNumber;packet.mode=(uint8_t)controlMode;packet.leftMotor=(int8_t)left;packet.rightMotor=(int8_t)right;if(emergency)packet.flags|=0x01;if(actionHeld)packet.flags|=0x02;esp_now_send(BROADCAST_MAC,reinterpret_cast<const uint8_t*>(&packet),sizeof(packet));}delay(2);}
