#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>

// commands constants
const byte OIL_PRESSURE_ON = B00011111;
const byte OIL_PRESSURE_OFF = B00010000;
const byte BATTERY_ON = B00101111;
const byte BATTERY_OFF = B00100000;
const byte PARKING_BRAKE_ON =  B00111111;
const byte PARKING_BRAKE_OFF = B00110000;
const byte BRAKES_OIL_ON = B01001111;
const byte BRAKES_OIL_OFF = B01000000;
const byte TURNING_SIGNS_ON = B01011111;
const byte TURNING_SIGNS_OFF = B01010000;
const byte SPARK_PLUGS_ON = B01111111;
const byte SPARK_PLUGS_OFF = B01110000;
const byte ABS_ANOMALY_ON = B10001111;
const byte ABS_ANOMALY_OFF = B10000000;
const byte HIGH_BEAM_ON = B10011111;
const byte HIGH_BEAM_OFF = B10010000;
const byte SPEED_PULSE = B10110000;
const byte RPM_PULSE = B10110100;
const byte DIESEL_VALUE = B11100000;
const byte TEMPERATURE_VALUE = B11000000;
const byte IGNITION_OFF = B10101010;
const byte IGNITION_ON = B10101011;
const byte TURN_OFF = B11111111;

byte previousValues[23];
int frequency = 0;
int frequencyCounter = 0;
int diesel = 0;
int dieselCounter = 0;
int temp = 0;
int tempCounter = 0;

//Pin assignments
int pinHighBeam = CONTROLLINO_A0; //branco1
int pinTurningSigns = CONTROLLINO_A1;//rosa1
int pinABS = CONTROLLINO_A2;//violeta1
int pinSparkPlugs = CONTROLLINO_A3;//preto1
int pinBrakesOil = CONTROLLINO_A4;//cinza1
int pinParkingBrake = CONTROLLINO_A5;//amarelo1
int pinBattery = CONTROLLINO_A6;//castanho1
int pinOilPressure = CONTROLLINO_A7;//azul1
int pinDiesel = CONTROLLINO_A8;//verde
int pinTemperature = CONTROLLINO_A9;//vermelho
int pinIgnitionPi = CONTROLLINO_A10;//laranja
int pinSpeed = CONTROLLINO_A11;//transparente
int pinRpm = CONTROLLINO_A12;//azul2
int pinBatteryVoltage = CONTROLLINO_A13;

int speedFrequency = 0;
int counter = 0;
int pinSpeedPreviousValue = 0;
float sp33d = 0;
int speedCounter = 0;

//Arduino Ethernet Config
byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x12
};
IPAddress ip(192, 168, 10, 69);
IPAddress remoteIp(192, 168, 10, 70);
IPAddress gateway(192, 168, 10, 252);
IPAddress subnet(255, 255, 255, 0);

EthernetUDP udp;

void setup() {
  //attachInterrupt(4, readPinSpeed, RISING);
  pinMode(pinOilPressure, INPUT);
  pinMode(pinBattery, INPUT);
  pinMode(pinParkingBrake, INPUT);
  pinMode(pinBrakesOil, INPUT);
  pinMode(pinTurningSigns, INPUT);
  pinMode(pinSparkPlugs, INPUT);
  pinMode(pinABS, INPUT);
  pinMode(pinHighBeam, INPUT);
  pinMode(pinSpeed, INPUT);
  pinMode(pinRpm, INPUT);
  pinMode(pinDiesel, INPUT);
  pinMode(pinTemperature, INPUT);
  pinMode(pinBatteryVoltage, INPUT);
  pinMode(pinIgnitionPi, INPUT);

  previousValues[pinOilPressure] = 0;
  previousValues[pinBattery] = 0;
  previousValues[pinParkingBrake] = 0;
  previousValues[pinBrakesOil] = 0;
  previousValues[pinTurningSigns] = 0;
  previousValues[pinSparkPlugs] = 0;
  previousValues[pinABS] = 0;
  previousValues[pinHighBeam] = 0;
  previousValues[pinSpeed] = 0;
  previousValues[pinRpm] = 0;
  previousValues[pinIgnitionPi] = 0;

  Ethernet.begin(mac, ip, gateway, subnet);
  udp.begin(23);
}

void loop() {
  readPins();
}

void readPinOilPressure()
{
  int value = digitalRead(pinOilPressure);
  if (value == 0 )
    udpwriteByte(OIL_PRESSURE_ON);
  else if (value == 1)
    udpwriteByte(OIL_PRESSURE_OFF);
}

void readPinBattery()
{
  int value = analogRead(pinBattery);
  if (value < 850)
    udpwriteByte(BATTERY_ON);
  else
    udpwriteByte(BATTERY_OFF);
}

void readPinParkingBrake()
{
  int value = digitalRead(pinParkingBrake);
  if (value == 0)
    udpwriteByte(PARKING_BRAKE_ON);
  else
    udpwriteByte(PARKING_BRAKE_OFF);

  previousValues[pinParkingBrake] = value;
}

void readPinBrakesOil()
{
  int value = digitalRead(pinBrakesOil);
  if (value == 0 )
    udpwriteByte(BRAKES_OIL_ON);
  else if (value == 1 )
    udpwriteByte(BRAKES_OIL_OFF);
}

void readPinTurningSigns()
{
  int value = digitalRead(pinTurningSigns);
  if (value == 0)
    udpwriteByte(TURNING_SIGNS_ON);
  else if (value == 1)
    udpwriteByte(TURNING_SIGNS_OFF);
}

void readPinSparkPlug()
{
  int value = digitalRead(pinSparkPlugs);
  if (value == 0)
    udpwriteByte(SPARK_PLUGS_ON);
  else if (value == 1)
    udpwriteByte(SPARK_PLUGS_OFF);
  previousValues[pinSparkPlugs] = value;
}

void readPinABS()
{
  int value = digitalRead(pinABS);
  if (value == 0 && previousValues[pinABS] == 1)
    udpwriteByte(ABS_ANOMALY_ON);
  else if (value == 1 && previousValues[pinABS] == 0)
    udpwriteByte(ABS_ANOMALY_OFF);
  previousValues[pinABS] = value;
}

void readPinHighBeam()
{
  int value = digitalRead(pinHighBeam);
  if (value == 1 && previousValues[pinHighBeam] == 0)
  {
    udpwriteByte(HIGH_BEAM_ON);
  }
  else if (value == 0 && previousValues[pinHighBeam] == 1)
    udpwriteByte(HIGH_BEAM_OFF);

  previousValues[pinHighBeam] = value;
}

void reapPinIgnition()
{
  int value = digitalRead(pinIgnitionPi);

  if (value == 0 && analogRead(pinBatteryVoltage) <= 600)
  {
    udpwriteByte(TURN_OFF);
  }

  if (value == 1 && previousValues[pinIgnitionPi] == 0)
  {
    udpwriteByte(IGNITION_ON);
  }
  else if (value == 0 && previousValues[pinIgnitionPi] == 1)
  {
    udpwriteByte(IGNITION_OFF);
  }
  previousValues[pinIgnitionPi] = value;
}

void readPinSpeed()
{
  if (speedCounter >= 15) {
    writeComplexCommand(SPEED_PULSE, (int)(sp33d / speedCounter));
    sp33d = 0;
    speedCounter = 0;
  } else {
    unsigned long pulseTime = pulseIn(pinSpeed, LOW, 10000);
    int frequencyInstant = 500000 / pulseTime;
    if (frequencyInstant > 25) {
      sp33d += (float) (frequencyInstant * 38) / (float) 210;
      speedCounter++;
    }
  }
}

void readPinRpm()
{
  unsigned long pulseTime = pulseIn(pinRpm, LOW, 7000);
  if (pulseTime == 0) {
    writeComplexCommand(RPM_PULSE, 0);
  } else {
    int frequencyInstant = 500000 / pulseTime;
    writeComplexCommand(RPM_PULSE, frequencyInstant);
  }
}

void readPinDiesel()
{
  writeComplexCommand(DIESEL_VALUE, analogRead(pinDiesel));
}

void readPinTemperature()
{
  writeComplexCommand(TEMPERATURE_VALUE, analogRead(pinTemperature));
}

void readPins()
{
  reapPinIgnition();
  readPinTurningSigns();
  readPinOilPressure();
  readPinBattery();
  readPinParkingBrake();
  readPinBrakesOil();
  readPinSparkPlug();
  readPinABS();
  readPinHighBeam();
  readPinSpeed();
  readPinRpm();
  readPinDiesel();
  readPinTemperature();
}

void writeComplexCommand(byte command, int value)
{
  byte complexCommand[3];
  complexCommand[0] = command;
  complexCommand[1] = value & 0xFF;
  complexCommand[2] = (value >> 8) & 0xFF;
  udpwrite(complexCommand, 3);
}

void udpwrite(byte* b, int sz)
{

  udp.beginPacket(remoteIp, 9887);
  //sprintf(ReplyBuffer,"%s",b);
  udp.write(b, sz); //ReplyBuffer);
  udp.endPacket();
}

void udpwriteByte(byte b)
{
  byte response[] = {b, 1};
  udp.beginPacket(remoteIp, 9887);
  udp.write(response, 2); //ReplyBuffer);
  udp.endPacket();
}
