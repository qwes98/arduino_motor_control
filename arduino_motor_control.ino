#define FOSC 16000000
#define BAUD 1000000

#define NUMBER_DXL 2

unsigned int MYUBRR = 0;

// flag
bool send_data_to_matlab = false;
bool interrupt_on = true;
bool infinite_mode = false;
bool start_flag = true;

// counter
unsigned long ISRcnt = 0;    // WARNING: ISRcnt를 int로 하면 매트랩 정지 현상 생김
int motorCnt[NUMBER_DXL] = {0};
int motorCntEnd[NUMBER_DXL];

// degree
int pastDegree[NUMBER_DXL];
unsigned int goalDegree[NUMBER_DXL];  // 지금 움직여야 하는 모터 각도
unsigned int curDegreeBuf[NUMBER_DXL] = {0};

// packet buffer
unsigned char readpacketBuf[20];

// Values from matlab
unsigned int motorID[NUMBER_DXL] = {1, 2};
int inputDegree[NUMBER_DXL];
int inputFreq[NUMBER_DXL];

// pin number
int ledPin = 13;
int controlPin = 4;

void setup() {
  pinMode(controlPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(1000000);    // matlab <-> arduino
  Serial1.begin(1000000);   // arduino <-> motor

  //interrupt 설정
  noInterrupts();
  TCCR1A = 0; //timer counter control register 초기화
  TCCR1B = 0;
  TCNT1 = 0; //count

  OCR1A = 15999; //0부터 세기 때문에 16000이 아니라 15999이다./pwm에 관련된 함수
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);

  // Arduino Mega의 TX1, RX1 사용하기 위함 (모터와 통신)
  UCSR1A |= (1 << U2X1); //U2X0-->통신속도를 2배로 해주는 세팅용 비트
  UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
  UCSR1C |= (1 << UCSZ11) | (1 << UCSZ01);
  MYUBRR = (FOSC / (8 * BAUD)) - 1;

  UBRR1H = (unsigned char)(MYUBRR << 8); //상위 비트를 저장(8자리를 넘어간다면 그것을 상위비트에 저장
  UBRR1L = (unsigned char)(MYUBRR); //하위 비트 저장(8자리를 넘어가는 것은 버림)
 
  sei();
  interrupts();

  bulkRead();

}

/* 모터 다음값 결정 */
double calMotorValue(double goal, double fre, int cnt, double current)
{
  return double(current + ((goal-current)/2.0)*(1-cos((2*3.14/(fre*10.0))*cnt)));
}

void USART_Transmit_for_1(unsigned char SEND)
{
  while(!(UCSR1A & (1 << UDRE1)));
  UDR1 = SEND;
}

/* 모터에 Syncwrite 패킷을 생성하여 보냄 */
void writeMotor()
{
  unsigned char writepacket[20];

  writepacket[0] = 0xFF;
  writepacket[1] = 0xFF;
  writepacket[2] = 0xFE;
  writepacket[3] = 0x0A;    // length   // 주의!! Hexa
  writepacket[4] = 0x83;    // instruction
  writepacket[5] = 0x1E;    // start address
  writepacket[6] = 0x02;    // data length
  writepacket[7] = byte(motorID[0]);   // first ID
  writepacket[8] = byte(goalDegree[0]);
  writepacket[9] = byte((goalDegree[0] & 0xff00) >> 8);
  writepacket[10] = byte(motorID[1]);  // second ID
  writepacket[11] = byte(goalDegree[1]);
  writepacket[12] = byte((goalDegree[1] & 0xff00) >> 8);
  
  unsigned int sum = 0;
  
  for(int i = 2; i < 13; i++)
  {
    sum += writepacket[i];
  }
  sum = ~byte(sum & 0x00FF);
  writepacket[13] = sum;
  
  for(int ii = 0; ii < 14; ii++)
  {
    USART_Transmit_for_1(writepacket[ii]);
    //Serial.println(writepacket[ii]);
  }
}

/* 모터에 bulkread 패킷을 생성하여 보냄 */
void bulkRead()
{
  unsigned char writepacket[20];

  writepacket[0] = 0xFF;
  writepacket[1] = 0xFF;
  writepacket[2] = 0xFE;
  writepacket[3] = 0x09;   // length
  writepacket[4] = 0x92;   
  writepacket[5] = 0x00;
  writepacket[6] = 0x02;   // first module data length
  writepacket[7] = byte(motorID[0]);   // first module id
  writepacket[8] = 0x24;   // data starting address
  writepacket[9] = 0x02;
  writepacket[10] = byte(motorID[1]);
  writepacket[11] = 0x24;

  unsigned int sum = 0;
  for(int i = 2; i < 12; i++)
  {
    sum += writepacket[i];
  }
  sum = ~byte(sum);
  writepacket[12] = sum;
  digitalWrite(controlPin, HIGH);
    
  for(int ii = 0; ii < 13; ii++)
  {
    USART_Transmit_for_1(writepacket[ii]);
  }

  delayMicroseconds(30);  // 리턴 패킷이 모두 제대로 들어오기 위한 시간 마련
  digitalWrite(controlPin, LOW);
}

void blinkLed()
{
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000); 
}

/* 매트랩으로부터 값을 받아옴 */
void data_reading_from_matlab()
{
  int i;
  int MID[20];

  if(Serial.available() > 0)
  {
    char ID1 = Serial.read();
    int ID = ID1 - '0';

    if(ID == 73)
    {
      interrupt_on = false;
    }
    else if(ID == 74)
    {
      interrupt_on = true;
    }
    else if(ID == 61)   // 'm' -> mode (0: finite, 1: infinite mode, 2: stop mode)
    {
      for(int i = 0; i < 1;)
      {
        if(Serial.available() > 0)
        {
          char ID1 = Serial.read();
          int ID = ID1 - '0';
          
          if(ID == 0)
          {
            infinite_mode = false;
          }
          else if(ID == 1)
          {
            infinite_mode = true;
          }
          else if(ID == 2)
          {
            for(int i = 0; i < NUMBER_DXL; i++)
            {
              pastDegree[i] = goalDegree[i];
              motorCnt[i] = motorCntEnd[i];
              infinite_mode = false;
            }
          }
          i++;
        }
      }
    }
    else if(ID == 49 || ID == 52)
    {
      for(i = 0; i < 2; )
      {
        if(Serial.available() > 0)
        {
          char ID1 = Serial.read();
          int ID = ID1 - '0';
          MID[i] = ID;
          i++;              // WARNING: 이 구문을 for문 선언안에 넣게 되면 패킷을 받아 이곳이 호출되기 전에 빠져나간다.
        }
      }

      int idx;
      if(ID == 49)
      {
        idx = 0;
      }
      else if(ID == 52)
      {
        idx = 1;
      }
      motorID[idx] = (MID[0]*10) + (MID[1]*1);

    }
    else if(ID == 50 || ID == 53)
    {
      for(i = 0; i < 4;)
      {
        if(Serial.available() > 0)
        {
          char ID1 = Serial.read();
          int ID = ID1 - '0';
          MID[i] = ID;
          i++;
        }
      }

      int idx;
      if(ID == 50)
      {
        idx = 0;
      }
      else if(ID == 53)
      {
        idx = 1;
      }

      inputDegree[idx] = (MID[0]*1000) + (MID[1]*100) + (MID[2]*10) + (MID[3]*1);
      motorCnt[idx] = 0;    // FIXME: 'z' 읽은 다음에 초기화?? (최대한 늦게?)
      
    }
    else if(ID == 51 || ID == 54)
    {
      for(i = 0; i < 4; )
      {
        if(Serial.available() > 0)
        {
          char ID1 = Serial.read();
          int ID = ID1 - '0';
          MID[i] = ID;
          i++;
        }
      }
      
      int idx;
      if(ID == 51)
      {
        idx = 0;
      }
      else if(ID == 54)
      {
        idx = 1;
      }
      
      inputFreq[idx] = (MID[0]*1000) + (MID[1]*100) + (MID[2]*10) + (MID[3]*1);
      motorCntEnd[idx] = 5 * inputFreq[idx];   // 1 - cos의 반주기
    }
    /*
    if(motorID[0] == 1 && inputDegree[0] == 1000 && inputFreq[0] == 1000
    && motorID[1] == 2 && inputDegree[1] == 2000 && inputFreq[1] == 2000)
    {
      blinkLed();
    }
    */
  }
}

/* 모터로부터 리턴패킷을 받아옴 */
void data_reading_from_motor_buf()
{
  if(Serial1.available() > 0)
  {
    
    if(Serial1.read() == 0xFF && Serial1.peek() == 0xFF)  // 두번째 peek으로 안하고 read로 하면 문제 생김
    {
      
      for(int i = 0; i < 7;)
      {
        if(Serial1.available() > 0)
        {
          readpacketBuf[i] = Serial1.read();
          
          if(i == 1)
          {
            if(!(readpacketBuf[1] == 1 || readpacketBuf[1] == 2))
            {
              break;
            }
          }
          //Serial.println(readpacketBuf[i]);
          i++;
        }
      }
      //Serial.println("-----");
      unsigned char sumOfPacket = 0;

          
      for(int i = 1; i < 6; i++)
      {
        sumOfPacket += readpacketBuf[i];
      }
      sumOfPacket = ~byte(sumOfPacket);   // 다시 넣어주어야 ~byte연산한 비트들을 unsigned char로 해석하게 되고 비교시 문제가 없음

      int tmp = readpacketBuf[5] << 8;
      tmp = tmp + readpacketBuf[4];
      if(sumOfPacket == readpacketBuf[6] && tmp > 0 && tmp < 4096)
      {
        // ID == 1
        if(readpacketBuf[1] == 1)
        {
          curDegreeBuf[0] = tmp;
          // 처음에 pastDegree, goalDegree 초기화
          if(start_flag == true)
          {
            pastDegree[0] = tmp;
            goalDegree[0] = (double)tmp;
          }
        }
        // ID == 2
        else if(readpacketBuf[1] == 2)
        {
          curDegreeBuf[1] = tmp;
          // 처음에 pastDegree, goalDegree 초기화
          if(start_flag == true)
          {
            pastDegree[1] = tmp;
            goalDegree[1] = (double)tmp;
            start_flag = false;
          }
        }
      }
    } 
  }
}

/* 매트랩에 모터의 현재 위치 값 보내기 */
void data_sending_to_matlab()
{
  for(int i = 0; i < NUMBER_DXL; i++)
  {
    Serial.println(round(curDegreeBuf[i]*360.0/4096));
  }
  send_data_to_matlab = false;
}

/* 모터 컨트롤을 위한 인터럽트 */
ISR(TIMER1_COMPA_vect)
{
  ISRcnt++;
  if(interrupt_on == true)
  {
    
  if(ISRcnt % 5 == 1)
  {
    digitalWrite(controlPin, HIGH);
 
    for(int i = 0; i < NUMBER_DXL; i++)
    {
      if(infinite_mode == true)
      {
        motorCnt[i]+=5;
        
        double tmp_goal = calMotorValue(inputDegree[i], inputFreq[i], motorCnt[i], pastDegree[i]);
        goalDegree[i] = tmp_goal;
      
      }
      else
      {
       
      if(motorCnt[i] < motorCntEnd[i])   // ISR -> 5ms마다 호출 => goal을 반주기로 나눠서 여러번 보냄?
      {
        motorCnt[i]+=5;    // 5씩 증가를 시켜야 함 (이 if문에 5초에 한번씩 들어오기 때문)
        
        double tmp_goal = calMotorValue(inputDegree[i], inputFreq[i], motorCnt[i], pastDegree[i]);
        goalDegree[i] = tmp_goal;
      } 
      else
      {
        // 다 움직인 후 pastDegree값 현재값(현재 goal)로 초기화
        if(start_flag == false)
        { 
          pastDegree[i] = goalDegree[i];
        }
        Serial.flush();
      }
      
      } // infinite_mode else end
    }
    writeMotor();
  }
  else if(ISRcnt % 5 == 3)
  {
    bulkRead();
  }
  }
  
  // 50ms마다 한번 씩 매트랩으로 값을 쏴준다
  if(ISRcnt % 50 == 10)   // bulkRead하는 시점과 조금 텀이 있어야!
  {
    send_data_to_matlab = true;
  }
 
}

void loop() 
{
  data_reading_from_matlab();

  data_reading_from_motor_buf();

  if(send_data_to_matlab == true)
  {
    data_sending_to_matlab();
  }
}
