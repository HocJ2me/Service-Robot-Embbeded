#include <Arduino.h>
#include <ArduinoModbus.h>
#include <ArduinoRS485.h>
//witmotion WT901C
#include "REG.h"
#include "wit_c_sdk.h"
#include "math.h"

#define FRONT_MOTOR_ID  1
#define LEFT_MOTOR_ID   2
#define RIGHT_MOTOR_ID  3

#define DEG_TO_RAD (M_PI/180)
#define c32    (-sqrt(3)/2)
uint16_t CTRL_CMD = 0x2000;
uint16_t SPEED_ADDR = 0x2001;
uint16_t ACC_ADDR = 0x2003; // gia toc 
uint16_t DEC_ADDR = 0x2004; // giam toc
uint16_t dir[3]={0x01,0x01,0x01};

//IMU
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};

int i;
float fAcc[3], fGyro[3], fAngle[3];
float offSet_z;
float x=0, Kp=2.0, Ki=0.01, Kd=0.09, P=0.0, I=0.0, D=0.0, Err=0.0, preErr=0.0;
int speed1, speed2, speed3;
uint8_t dir1, dir2, dir3;
#define DEG_TO_RAD  (float)(M_PI/180)
//
#define SAMPLING_TIME_MS 10
#define SAMPLING_TIME_S  (float)(SAMPLING_TIME_MS/1000)
#define INV_SAMPLING_TIME (float)(1000/SAMPLING_TIME_MS)
#define TEST 0
char BT_cmd, last_BT_cmd;
void setup(){
    Serial.begin(9600);    //Serial Monitor
    Serial1.begin(9600); //Serial bluetooth
    if (!ModbusRTUClient.begin(57600)) {
      Serial.println("Failed to start Modbus RTU Client!");
      while (1);
    }
    Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, 0x05);
    Write_Data_Modbus(FRONT_MOTOR_ID, ACC_ADDR, 8);
    Write_Data_Modbus(FRONT_MOTOR_ID, DEC_ADDR, 8);
    delay(1);
    Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x05);
    Write_Data_Modbus(LEFT_MOTOR_ID, ACC_ADDR, 8);
    Write_Data_Modbus(LEFT_MOTOR_ID, DEC_ADDR, 8);
    delay(1);
    Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x05);
    Write_Data_Modbus(RIGHT_MOTOR_ID, ACC_ADDR, 8);
    Write_Data_Modbus(RIGHT_MOTOR_ID, DEC_ADDR, 8);
    delay(1000);

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	  WitSerialWriteRegister(SensorUartSend);
	  WitRegisterCallBack(SensorDataUpdata);
    WitDelayMsRegister(Delayms);
	  Serial.print("\r\n********************** wit-motion setup  ************************\r\n");
	  AutoScanSensor();
    Serial.println("Robot is Ready");

    read_IMU();
    Serial.print("Z IMU angle First: ");
    Serial.println(fAngle[2],3);
    offSet_z = fAngle[2];
}   

void Write_Data_Modbus(int id, int address, uint16_t value)
{
  if (!ModbusRTUClient.holdingRegisterWrite(id, address, value)) {
    //Serial.print("Failed to write coil! ");
    //Serial.println(ModbusRTUClient.lastError());
  }
  delay(3);
  //delayMicroseconds(1);
}
void robot_Stop(){
  speed1 =0; speed2=0; speed3=0;
  Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, 0x06);
  Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x06);
  Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x06);
  resetIMU();
}

// void robot_Run(int speed_max, int angle){
//     static int v1 = -c32*cos(angle*DEG_TO_RAD)-0.5*sin(angle*DEG_TO_RAD);
//     static int v2 = c32*cos(angle*DEG_TO_RAD)-0.5*sin(angle*DEG_TO_RAD);
//     static int v3 = sin(angle*DEG_TO_RAD);
//     if(v1>=0) dir[0]=0x01;
//     else dir[0]=0x02;
//     if(v2>=0) dir[1]=0x01;
//     else dir[1]=0x02;
//     if(v3>=0) dir[2]=0x01;
//     else dir[2]=0x02;
//     Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, dir[0]);
//     Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, dir[1]);
//     Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, dir[2]);
//     Write_Data_Modbus(FRONT_MOTOR_ID, SPEED_ADDR, v1*speed_max);
//     Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, v2*speed_max);
//     Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, v3*speed_max);

// }
// void serialEvent1(){
//   if(Serial1.available()){
//     BT_cmd = Serial1.read();
//   }
// }
static unsigned long prev_imu_t = 0;
static bool robot_run_state = 0, imu_state = 1, rotate_state =0;
int max_Speed_test = 200;
void loop(){
      //Serial.write(SerialBT.read());
      if(millis() - prev_imu_t > 10){
        if(imu_state == 1){
          read_IMU();
        }
        prev_imu_t = millis();
        if(robot_run_state){
          calculate_Speed();
        }
      }

      #if !TEST
      if(Serial1.available()){
        BT_cmd = Serial1.read();
      }
      // if(BT_cmd !=last_BT_cmd){
        Serial.print(BT_cmd);
      //   last_BT_cmd = BT_cmd;
        if(BT_cmd!='0') robot_run_state = 1;
        else robot_run_state = 0;
        switch (BT_cmd)
        {
          case '3': 
          //forward
          robot_Run(max_Speed_test,0);
          // Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, 150);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, 150);
          // Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x01);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x02);
          // Serial.println("FORWARD");
          //robot_Run(400,90);
          break;

          case '4': 
          //backward
          robot_Run(max_Speed_test,-180);
          // Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, 150);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, 150);
          // Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x02);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x01);
          // Serial.println("BACKWARD");
          //robot_Run(400,270);
          break;

          case '6': 
          //rotate left
          robot_Rotate(100);
          // Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(FRONT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x01);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x01);
          // Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, 0x01);
          // Serial.println("FORWARD");
          //robot_Run(400,90);
          break;

          case '5': 
          //rotate right
          robot_Rotate(-100);
          // Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x02);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x02);
          // Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, 0x02);
          // Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(FRONT_MOTOR_ID, SPEED_ADDR, 100);
          // Serial.println("BACKWARD");
          //robot_Run(400,270);
          break;

          case 'B': 
          robot_Run(max_Speed_test,90);
          // Write_Data_Modbus(FRONT_MOTOR_ID, SPEED_ADDR, 200);
          // Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, 0x01);
          // Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x02);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x02);
          // Serial.println("RIGHT");
          //robot_Run(400,0);
          break;

          case 'D': 
          robot_Run(max_Speed_test,-90);
          // Write_Data_Modbus(FRONT_MOTOR_ID, SPEED_ADDR, 200);
          // Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, 100);
          // Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, 0x02);
          // Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x01);
          // Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x01);
          // Serial.println("LEFT");
          //robot_Run(400,180);
          break;
          
          case 'A':
          robot_Run(max_Speed_test,-45);
          break;

          case 'E':
          robot_Run(max_Speed_test,45);     
          break;

          case 'C':
          robot_Run(max_Speed_test,-135);     
          break;

          case 'F':
          robot_Run(max_Speed_test,135);     
          break;

          case '0':
          // Serial.println("STOP");
          robot_run_state = 0;
          robot_Stop();
          break;
          case 'P':
          //ON IMU && reset IMU
          imu_state = 1;
          read_IMU();
          offSet_z = fAngle[2];
          break;
          case 'U':
          //OFF IMU
          imu_state = 0;
          fAngle[2] = 0;
          offSet_z = 0;
          break;
        default:
          robot_run_state = 0;
          robot_Stop();
          break;
        }
        if(robot_run_state!=0){
          Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, speed3);
          Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, speed2);
          Write_Data_Modbus(FRONT_MOTOR_ID, SPEED_ADDR, speed1);

          Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, dir3);
          Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, dir2);
          Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, dir1);
        } 
      // }
      #else
        Write_Data_Modbus(FRONT_MOTOR_ID, CTRL_CMD, 0x02);
        Write_Data_Modbus(LEFT_MOTOR_ID, CTRL_CMD, 0x02);
        Write_Data_Modbus(RIGHT_MOTOR_ID, CTRL_CMD, 0x02);
        Write_Data_Modbus(FRONT_MOTOR_ID, SPEED_ADDR, 150);
        Write_Data_Modbus(LEFT_MOTOR_ID, SPEED_ADDR, 150);
        Write_Data_Modbus(RIGHT_MOTOR_ID, SPEED_ADDR, 150);
      #endif 
}   
void resetIMU(){
  imu_state = 1;
  read_IMU();
  offSet_z = fAngle[2];
}
void calculate_Speed(void){
  // float angle_z = fAngle[2];
  Err = (float)(fAngle[2]-offSet_z);
  P = Kp*Err;
  I = I + 0.5*Ki*SAMPLING_TIME_S*(Err+preErr);
  D = Kd*(Err-preErr)*INV_SAMPLING_TIME;
  x = (int) P+I+D;

  Serial.print(" Anglez: "); Serial.print(fAngle[2]);
  Serial.print(" Zeroz: "); Serial.print(offSet_z);
  Serial.print(" Err: "); Serial.print(Err);
  Serial.print(" P: "); Serial.print(P);
  Serial.print(" I: "); Serial.print(I);
  Serial.print(" D: "); Serial.print(D);
  Serial.print(" x: "); Serial.println(x);

  Serial.print(" v1: "); Serial.print(speed1);
  Serial.print(" v2: "); Serial.print(speed2);
  Serial.print(" v3: "); Serial.println(speed3);

  if(speed1>=0) dir1=0x01; else {dir1=0x02; speed1=-speed1;}
  if(speed2>=0) dir2=0x01; else {dir2=0x02; speed2=-speed2;}
  if(speed3>=0) dir3=0x01; else {dir3=0x02; speed3=-speed3;}

  preErr = Err;
}

void robot_Run(int maxSpeed, int angle){
  imu_state = 1;
  rotate_state = 0;
  speed1 = (int) maxSpeed*sin(angle*DEG_TO_RAD) + x*0.7 ;
  speed2 = (int) -maxSpeed*cos((angle-30)*DEG_TO_RAD) + x;
  speed3 = (int) maxSpeed*cos((angle+30)*DEG_TO_RAD) + x;
  
  if(abs(speed1) > maxSpeed){
    if(speed1>=0) speed1=maxSpeed;
    else speed1=-maxSpeed;
  } 

  if(abs(speed2) > maxSpeed){
    if(speed2>=0) speed2=maxSpeed;
    else speed2=-maxSpeed;
  } 

    if(abs(speed3) > maxSpeed){
    if(speed3>=0) speed3=maxSpeed;
    else speed3=-maxSpeed;
  } 

  
  // Serial.print(" Err: "); Serial.print(Err);
  // Serial.print(" P: "); Serial.print(P);
  // Serial.print(" I: "); Serial.print(I);
  // Serial.print(" D: "); Serial.print(D);
  // Serial.print(" x: "); Serial.println(x);

  // Serial.print(" v1: "); Serial.print(speed1);
  // Serial.print(" v2: "); Serial.print(speed2);
  // Serial.print(" v3: "); Serial.println(speed3);

  if(speed1>=0) dir1=0x01; else {dir1=0x02; speed1=-speed1;}
  if(speed2>=0) dir2=0x01; else {dir2=0x02; speed2=-speed2;}
  if(speed3>=0) dir3=0x01; else {dir3=0x02; speed3=-speed3;}


}

void robot_Rotate(int maxSpeed){
  rotate_state = 1;
  imu_state = 0;
  if(maxSpeed>0){
    dir1 = 0x01; dir2 = 0x01; dir3=0x01;
  }
  else{
    dir1 = 0x02; dir2 = 0x02; dir3=0x02;
  }
  speed1 = abs(maxSpeed); speed2=abs(maxSpeed); speed3=abs(maxSpeed);
}

// IMU WT901C function
void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}
}
static void ShowHelp(void)
{
	Serial.print("\r\n************************	 WIT_SDK_DEMO	************************");
	Serial.print("\r\n************************          HELP           ************************\r\n");
	Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	Serial.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  Serial.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  Serial.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  Serial.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
	Serial.print("******************************************************************************\r\n");
}
static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	if(WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	if(WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	if(WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial3.begin(c_uiBaud[WIT_BAUD_115200]);
                Serial.print(" 115200 Baud rate modified successfully\r\n");
              }
			break;
		case 'b':	if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial3.begin(c_uiBaud[WIT_BAUD_9600]); 
                Serial.print(" 9600 Baud rate modified successfully\r\n");
              }
			break;
		case 'r': if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)  Serial.print("\r\nSet Baud Error\r\n");
			        else Serial.print("\r\nSet Baud Success\r\n");
			break;
		case 'R':	if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else Serial.print("\r\nSet Baud Success\r\n");
			break;
    case 'C': if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'c': if(WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
		case 'h':	ShowHelp();
			break;
		default :break;
	}
	s_cCmd = 0xff;
}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  Serial3.write(p_data, uiSize);
  Serial3.flush();
}
static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}
static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
	{
		Serial3.begin(c_uiBaud[i]);
    Serial3.flush();
		iRetry = 2;
		s_cDataUpdate = 0;
		do
		{
			WitReadReg(AX, 3);
			delay(200);
      while (Serial3.available())
      {
        WitSerialDataIn(Serial3.read());
      }
			if(s_cDataUpdate != 0)
			{
				Serial.print(c_uiBaud[i]);
				Serial.print(" baud find sensor\r\n\r\n");
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	Serial.print("can not find sensor\r\n");
	Serial.print("please check your connection\r\n");
}

void read_IMU(void){
  while (Serial3.available())
  {
    WitSerialDataIn(Serial3.read());
  }
  if(s_cDataUpdate)
	{
		for(i = 0; i < 3; i++)
		{
			fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
			fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
			fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
		}

    if(s_cDataUpdate & ANGLE_UPDATE)
		{
			// Serial.print("angle x: ");
			// Serial.print(fAngle[0], 3);
			// Serial.print("y: ");
			// Serial.print(fAngle[1], 3);
			// Serial.print("z: ");
			// Serial.print(fAngle[2], 3);
			// Serial.print("\r\n");
			s_cDataUpdate &= ~ANGLE_UPDATE;
		}
    s_cDataUpdate = 0;
  }
}