/* ROBÔ V6.1 — ESP32 WROOM-32 + ESP-NOW */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_arduino_version.h>
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14
#define ENB 13
#define TRIG_PIN 18
#define ECHO_PIN 19
#define LED_PIN 2
const bool INVERT_LEFT_MOTOR=false,INVERT_RIGHT_MOTOR=false,LED_HIGH_ON=true,OBSTACLE_BRAKE_ENABLED=true; const float SAFETY_DISTANCE_CM=20.0f; const unsigned long FAILSAFE_MS=450,SENSOR_INTERVAL_MS=80,TELEMETRY_INTERVAL_MS=300; const int PWM_FREQ=1000,PWM_RES=8,PWM_MAX=255,PWM_MIN_MOVEMENT=105;
static const uint32_t MAGIC_CONTROL=0x524F424F,MAGIC_TELEM=0x54454C45;
struct __attribute__((packed)) ControlPacket{uint32_t magic;uint16_t seq;uint8_t mode;int8_t leftMotor;int8_t rightMotor;uint8_t flags;};
struct __attribute__((packed)) TelemetryPacket{uint32_t magic;uint16_t lastSeq;float distanceCm;uint8_t obstacle;uint8_t failsafe;uint8_t emergency;};
portMUX_TYPE packetMux=portMUX_INITIALIZER_UNLOCKED; ControlPacket latestPacket={}; volatile bool newPacket=false; volatile unsigned long lastPacketMs=0; unsigned long lastSensorMs=0,lastTelemetryMs=0,lastSerialMs=0; float distanceCm=999.0f; bool failsafeActive=true,emergencyActive=false,obstacleActive=false; uint16_t lastSequence=0; uint8_t controllerMac[6]={}; volatile bool controllerMacKnown=false,controllerPeerNeedsAdd=false;
#if ESP_ARDUINO_VERSION_MAJOR < 3
const int PWM_CH_A=0,PWM_CH_B=1;
#endif
void initPWM(){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
ledcAttach(ENA,PWM_FREQ,PWM_RES);ledcAttach(ENB,PWM_FREQ,PWM_RES);
#else
ledcSetup(PWM_CH_A,PWM_FREQ,PWM_RES);ledcSetup(PWM_CH_B,PWM_FREQ,PWM_RES);ledcAttachPin(ENA,PWM_CH_A);ledcAttachPin(ENB,PWM_CH_B);
#endif
}
void writePWMA(int value){value=constrain(value,0,PWM_MAX);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
ledcWrite(ENA,value);
#else
ledcWrite(PWM_CH_A,value);
#endif
}
void writePWMB(int value){value=constrain(value,0,PWM_MAX);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
ledcWrite(ENB,value);
#else
ledcWrite(PWM_CH_B,value);
#endif
}
int percentToPWM(int percent){percent=abs(percent);if(percent<=0)return 0;percent=constrain(percent,1,100);return map(percent,1,100,PWM_MIN_MOVEMENT,PWM_MAX);}
void setLed(bool on){digitalWrite(LED_PIN,LED_HIGH_ON?(on?HIGH:LOW):(on?LOW:HIGH));}
void setLeftMotor(int percent){percent=constrain(percent,-100,100);if(INVERT_LEFT_MOTOR)percent*=-1;int pwm=percentToPWM(percent);if(percent>0){digitalWrite(IN1,HIGH);digitalWrite(IN2,LOW);}else if(percent<0){digitalWrite(IN1,LOW);digitalWrite(IN2,HIGH);}else{digitalWrite(IN1,LOW);digitalWrite(IN2,LOW);}writePWMA(pwm);}
void setRightMotor(int percent){percent=constrain(percent,-100,100);if(INVERT_RIGHT_MOTOR)percent*=-1;int pwm=percentToPWM(percent);if(percent>0){digitalWrite(IN3,HIGH);digitalWrite(IN4,LOW);}else if(percent<0){digitalWrite(IN3,LOW);digitalWrite(IN4,HIGH);}else{digitalWrite(IN3,LOW);digitalWrite(IN4,LOW);}writePWMB(pwm);}
void stopMotors(){setLeftMotor(0);setRightMotor(0);} void drive(int left,int right){setLeftMotor(left);setRightMotor(right);}
float readDistance(){digitalWrite(TRIG_PIN,LOW);delayMicroseconds(2);digitalWrite(TRIG_PIN,HIGH);delayMicroseconds(10);digitalWrite(TRIG_PIN,LOW);unsigned long duration=pulseIn(ECHO_PIN,HIGH,26000);if(duration==0)return 999.0f;float d=duration*0.0343f/2.0f;if(d<2.0f||d>450.0f)return 999.0f;return d;}
void updateDistance(){if(millis()-lastSensorMs<SENSOR_INTERVAL_MS)return;lastSensorMs=millis();distanceCm=readDistance();obstacleActive=distanceCm<900.0f&&distanceCm<=SAFETY_DISTANCE_CM;}
void rememberControllerMac(const uint8_t*mac){bool changed=!controllerMacKnown;if(!changed){for(int i=0;i<6;i++)if(controllerMac[i]!=mac[i]){changed=true;break;}}if(changed){memcpy(controllerMac,mac,6);controllerMacKnown=true;controllerPeerNeedsAdd=true;}}
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t*info,const uint8_t*data,int len){if(info!=nullptr)rememberControllerMac(info->src_addr);
#else
void onDataRecv(const uint8_t*mac,const uint8_t*data,int len){rememberControllerMac(mac);
#endif
if(len!=sizeof(ControlPacket))return;ControlPacket packet;memcpy(&packet,data,sizeof(packet));if(packet.magic!=MAGIC_CONTROL)return;portENTER_CRITICAL(&packetMux);latestPacket=packet;newPacket=true;portEXIT_CRITICAL(&packetMux);lastPacketMs=millis();}
void ensureControllerPeer(){if(!controllerPeerNeedsAdd||!controllerMacKnown)return;controllerPeerNeedsAdd=false;if(esp_now_is_peer_exist(controllerMac))return;esp_now_peer_info_t peer={};memcpy(peer.peer_addr,controllerMac,6);peer.channel=0;peer.encrypt=false;esp_now_add_peer(&peer);}
void sendTelemetry(){if(!controllerMacKnown||millis()-lastTelemetryMs<TELEMETRY_INTERVAL_MS)return;lastTelemetryMs=millis();TelemetryPacket packet={};packet.magic=MAGIC_TELEM;packet.lastSeq=lastSequence;packet.distanceCm=distanceCm;packet.obstacle=obstacleActive?1:0;packet.failsafe=failsafeActive?1:0;packet.emergency=emergencyActive?1:0;esp_now_send(controllerMac,reinterpret_cast<const uint8_t*>(&packet),sizeof(packet));}
bool initEspNow(){WiFi.mode(WIFI_STA);WiFi.disconnect();delay(100);if(esp_now_init()!=ESP_OK)return false;esp_now_register_recv_cb(onDataRecv);return true;}
void applyControlPacket(const ControlPacket&packet){emergencyActive=(packet.flags&0x01)!=0;lastSequence=packet.seq;if(emergencyActive){stopMotors();return;}int left=packet.leftMotor,right=packet.rightMotor;bool tryingToAdvance=(left+right)>20&&(left>0||right>0);if(OBSTACLE_BRAKE_ENABLED&&obstacleActive&&tryingToAdvance){stopMotors();return;}drive(left,right);}
void setup(){Serial.begin(115200);delay(700);pinMode(IN1,OUTPUT);pinMode(IN2,OUTPUT);pinMode(IN3,OUTPUT);pinMode(IN4,OUTPUT);pinMode(ENA,OUTPUT);pinMode(ENB,OUTPUT);pinMode(TRIG_PIN,OUTPUT);pinMode(ECHO_PIN,INPUT);pinMode(LED_PIN,OUTPUT);initPWM();stopMotors();setLed(false);if(!initEspNow())while(true){setLed(!digitalRead(LED_PIN));delay(250);}Serial.println("Robo V6.1 pronto.");}
void loop(){updateDistance();ensureControllerPeer();ControlPacket packetCopy={};bool hasNew=false;portENTER_CRITICAL(&packetMux);if(newPacket){packetCopy=latestPacket;newPacket=false;hasNew=true;}portEXIT_CRITICAL(&packetMux);if(hasNew){failsafeActive=false;applyControlPacket(packetCopy);}if(millis()-lastPacketMs>FAILSAFE_MS){failsafeActive=true;stopMotors();}if(emergencyActive)setLed(((millis()/120)%2)==0);else if(failsafeActive)setLed(((millis()/450)%2)==0);else setLed(true);sendTelemetry();delay(2);}
