/**
 * 09.03 - 09.09 
 * 
 * 1. variable => array: OK
 * 2. matlab->arduino comm: OK 
 * 3. syncWrite: OK
 * 4. bulkRead:
 * 5. arduino->matlab:
 * 
 */

#define FOSC 16000000
#define BAUD 1000000

const int number_dxl = 2;
bool send_data_to_matlab = false;
bool interrupt_on = true;
bool read_from_buf = false;
bool start_flag = true;
unsigned long ISR_cnt = 0;    // ISR_cnt를 int로 하면 매트랩 정지 현상 생김
int Tcnt[number_dxl] = {0};
// TODO: pastDEGREE -> array
int pastDEGREE[number_dxl];
// TODO: pos 수정
unsigned int pos[number_dxl];  // 지금 움직여야 하는 모터 각도
// TODO: Mcount, MFRE -> Mcount[0] or Mcount[1]
int Mcount[number_dxl];
int MFRE[number_dxl];
unsigned int MYUBRR = 0;
unsigned char a[20];
unsigned char readpacket[20];
// TODO: curDegreeBuf -> array
unsigned int curDegreeBuf[number_dxl] = {0};

// Values from matlab
// TODO: MIDc, MMDEFREE, MMFRE -> MIDc[0] or MIDc[1]
unsigned int MIDc[number_dxl] = {1, 2};
int MMDEGREE[number_dxl];
int MMFRE[number_dxl];  // FIXME: have to initialize?

int ledpin = 13;
int controlpin = 4;

void setup() {
  // put your setup code here, to run once:

  pinMode(controlpin, OUTPUT);
  pinMode(ledpin, OUTPUT);
  Serial.begin(1000000);    // matlab <-> arduino
  Serial1.begin(1000000);   // arduino <-> motor

  noInterrupts();
  TCCR1A = 0; //timer counter control register 초기화
  TCCR1B = 0;
  TCNT1 = 0; //count

  OCR1A = 15999; //0부터 세기 때문에 16000이 아니라 15999이다./pwm에 관련된 함수
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);


  //  UCSR0A |= (1 << U2X0); //U2X0-->통신속도를 2배로 해주는 세팅용 비트
  //  UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
  //  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
  //  MYUBRR = (FOSC / (8 * BAUD)) - 1;
  //
  //  UBRR0H = (unsigned char)(MYUBRR << 8); //상위 비트를 저장(8자리를 넘어간다면 그것을 상위비트에 저장
  //  UBRR0L = (unsigned char)(MYUBRR); //하위 비트 저장(8자리를 넘어가는 것은 버림)

  // Arduino Mega의 TX1, RX1 사용하기 위함 (모터와 통신)
  UCSR1A |= (1 << U2X1); //U2X0-->통신속도를 2배로 해주는 세팅용 비트
  UCSR1B |= (1 << RXEN1) | (1 << TXEN1);
  UCSR1C |= (1 << UCSZ11) | (1 << UCSZ01);
  MYUBRR = (FOSC / (8 * BAUD)) - 1;

  UBRR1H = (unsigned char)(MYUBRR << 8); //상위 비트를 저장(8자리를 넘어간다면 그것을 상위비트에 저장
  UBRR1L = (unsigned char)(MYUBRR); //하위 비트 저장(8자리를 넘어가는 것은 버림)
 
  sei();
  interrupts();

  // pastDEGREE,pos 초기화
  digitalWrite(controlpin, HIGH);
  bulkRead();
  read_from_buf = true;   // bulkRead가 호출되어 리턴 패킷을 받았을때에만 loop에서 읽기 위해 플래그 사용 -> 매트랩 그래프 계단현상 해결
  delayMicroseconds(30);  // 리턴 패킷이 모두 제대로 들어오기 위한 시간 마련
  digitalWrite(controlpin, LOW);

}

int dynamixel_CF_movement(double degree, double fre, int cnt, double tmp)   //degree: 목표값, fre: 진행시간, cnt: x축정의역, tmp: 모터의 현재위치
{
  double answer = 0;  
  answer = tmp + ((degree-tmp)/2.0)*(1-cos((3.14*cnt)/(fre*10.0)));

  return answer;
}

void USART_Transmit_for_1(unsigned char SEND)
{
  while(!(UCSR1A & (1 << UDRE1)));
  UDR1 = SEND;
}

void writeMotor()
{
  // Write
  a[0] = 0xFF;
  a[1] = 0xFF;
  a[2] = 0xFE;
  a[3] = 0x0A;    // length   // 주의!! Hexa
  a[4] = 0x83;    // instruction
  a[5] = 0x1E;    // start address
  a[6] = 0x02;    // data length
  a[7] = byte(MIDc[0]);   // first ID
  a[8] = byte(pos[0]);
  a[9] = byte((pos[0] & 0xff00) >> 8);
  a[10] = byte(MIDc[1]);  // second ID
  a[11] = byte(pos[1]);
  a[12] = byte((pos[1] & 0xff00) >> 8);
  
  unsigned int sum = 0;
  int i;
  for(i = 2; i < 13; i++)
  {
    sum += a[i];
  }
  sum = ~byte(sum & 0x00FF);
  a[13] = sum;
  
  for(int ii = 0; ii < 14; ii++)
  {
    USART_Transmit_for_1(a[ii]);
    //Serial.println(a[ii]);
  }
}

void bulkRead()
{
  a[0] = 0xFF;
  a[1] = 0xFF;
  a[2] = 0xFE;
  a[3] = 0x09;   // length
  a[4] = 0x92;   
  a[5] = 0x00;
  a[6] = 0x02;   // first module data length
  a[7] = byte(MIDc[0]);   // first module id
  a[8] = 0x24;   // data starting address
  a[9] = 0x02;
  a[10] = byte(MIDc[1]);
  a[11] = 0x24;

  unsigned int sum = 0;
  for(int i = 2; i < 12; i++)
  {
    sum += a[i];
  }
  sum = ~byte(sum);
  a[12] = sum;
    
  for(int ii = 0; ii < 13; ii++)
  {
    USART_Transmit_for_1(a[ii]);
  }
}

void blinkLed()
{
  digitalWrite(ledpin, HIGH);
  delay(1000);
  digitalWrite(ledpin, LOW);
  delay(1000); 
}

// DONE for 3
void data_reading_from_matlab()
{
  int i;
  int MID[20];
  //int MDEGREE;

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
      MIDc[idx] = (MID[0]*10) + (MID[1]*1);

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

      MMDEGREE[idx] = (MID[0]*1000) + (MID[1]*100) + (MID[2]*10) + (MID[3]*1);
      Tcnt[idx] = 0;
      
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
      
      MFRE[idx] = (MID[0]*1000) + (MID[1]*100) + (MID[2]*10) + (MID[3]*1);
      MMFRE[idx] = MFRE[idx];
      Mcount[idx] = 5 * MMFRE[idx];   // 1 - cos의 반주기
    }
    /*
    if(MIDc[0] == 1 && MMDEGREE[0] == 1000 && MFRE[0] == 1000
    && MIDc[1] == 2 && MMDEGREE[1] == 2000 && MFRE[1] == 2000)
    {
      blinkLed();
    }
    */
  }
}

/* 
 *  HAVE TO DO
 *  problem
 *  1. 255가 두번, 세번 나올때가 있음
 *  2. id가 2인 값이 없음
 *  
 *  cf. 맨 처음 요청시 id1, id2 모터 위치값 잘 읽어옴
 */
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
          readpacket[i] = Serial1.read();
          i++;
          //Serial.println(readpacket[i]);
        }
      }
      //Serial.println("-----");
      unsigned char sumOfPacket = 0;
      
      for(int i = 1; i < 6; i++)
      {
        sumOfPacket += readpacket[i];
      }
      sumOfPacket = ~byte(sumOfPacket);   // 다시 넣어주어야 ~byte연산한 비트들을 unsigned char로 해석하게 되고 비교시 문제가 없음

      int tmp = readpacket[5] << 8;
      tmp = tmp + readpacket[4];
      if(sumOfPacket == readpacket[6] && tmp > 0 && tmp < 4096)
      {
        // ID == 1
        if(readpacket[1] == 1)
        {
          curDegreeBuf[0] = tmp;
          // 처음에 pastDEGREE, pos 초기화
          if(start_flag)
          {
            pastDEGREE[0] = tmp;
            pos[0] = (double)tmp;
          }
        }
        // ID == 2
        else if(readpacket[1] == 2)
        {
          curDegreeBuf[1] = tmp;
          // 처음에 pastDEGREE, pos 초기화
          if(start_flag)
          {
            pastDEGREE[1] = tmp;
            pos[1] = (double)tmp;
            start_flag = false;   // FIXME: 첫번째모터 초기화, 두번째모터 초기화 알고리즘 바꾸기
          }
        }
      }

      /*
      // 처음에 pastDEGREE, pos 초기화
      if(start_flag)
      {
        pastDEGREE[0] = tmp;
        pos[0] = tmp;
        start_flag = false;
      }
      */
    }
  }
  //Serial.println("============");
  read_from_buf = false;
}

void data_sending_to_matlab()
{
  // FIXME
  for(int i = 0; i < number_dxl; i++)
  {
    Serial.println(round(curDegreeBuf[i]*360.0/4096));
  }
  //Serial.println("-----");
  send_data_to_matlab = false;
}

ISR(TIMER1_COMPA_vect)
{
  ISR_cnt++;
  if(interrupt_on)
  {
    
  if(ISR_cnt % 5 == 1)
  {
    //motor_goal = dynamixel_CF_movement(MMDEGREE, MMFRE, ISR_cnt, pastDEGREE);
    digitalWrite(controlpin, HIGH);
 
    for(int i = 0; i < number_dxl; i++)
    {
      if(Tcnt[i] < Mcount[i])   // ISR -> 5ms마다 호출 => goal을 반주기로 나눠서 여러번 보냄?
      {
        Tcnt[i]+=5;    // 5씩 증가를 시켜야 함 (이 if문에 5초에 한번씩 들어오기 때문)
        // TODO: generalization
        double w;
        if((MMDEGREE[i] - pastDEGREE[i]) > 0)
        {
          // TODO: modify index
          w = pastDEGREE[i] + (MMDEGREE[i] - pastDEGREE[i])/2.0*(1-cos((2*3.14/(MMFRE[i]*10.0))*Tcnt[i])); 
        }
        else
        { 
          w = pastDEGREE[i] - (-1*(MMDEGREE[i] - pastDEGREE[i]))/2.0*(1-cos((2*3.14/(MMFRE[i]*10.0))*Tcnt[i]));
        }
        pos[i] = w;
      } 
      else
      {
        if(!start_flag)
        {
          pastDEGREE[i] = pos[i];
        }
        Serial.flush();
      }
    }
    writeMotor();
  }
  else if(ISR_cnt % 5 == 3)
  {
    digitalWrite(controlpin, HIGH);
    bulkRead();
    read_from_buf = true;   // bulkRead가 호출되어 리턴 패킷을 받았을때에만 loop에서 읽기 위해 플래그 사용 -> 매트랩 그래프 계단현상 해결
    delayMicroseconds(30);  // 리턴 패킷이 모두 제대로 들어오기 위한 시간 마련
    digitalWrite(controlpin, LOW);
  }
  }
  
  // 50ms마다 한번 씩 매트랩으로 값을 쏴준다
  if(ISR_cnt % 50 == 10)   // bulkRead하는 시점과 조금 텀이 있어야!
  {
    send_data_to_matlab = true;
  }
 
}

void loop() 
{
  // put your main code here, to run repeatedly:
  data_reading_from_matlab();

  if(read_from_buf)
  {
    data_reading_from_motor_buf();
  }

  if(send_data_to_matlab)
  {
    data_sending_to_matlab();
  }
}
