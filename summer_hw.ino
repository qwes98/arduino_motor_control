#define FOSC 16000000
#define BAUD 1000000

bool send_data_to_matlab = false;
int ISR_cnt = 0;
int Tcnt = 0;
int pastDEGREE;
unsigned int w;  // 지금 움직여야 하는 모터 각도
int Mcount;
int MFRE;
unsigned int MYUBRR = 0;
unsigned char a[20];
int readpacket[20];

// Values from matlab
unsigned int MIDc;
int MMDEGREE;
int MMFRE;  // FIXME: have to initialize?

int ledpin = 13;
int controlpin = 4;

void setup() {
  // put your setup code here, to run once:

  pinMode(controlpin, OUTPUT);
  pinMode(ledpin, OUTPUT);
  Serial.begin(1000000);
  Serial1.begin(1000000);

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

  // TODO: 현재 모터값 읽어와서 pastDEGREE 초기화
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

void writeMotor(unsigned int ID, unsigned int pos)
{
  // Write
  a[0] = 0xFF;
  a[1] = 0xFF;
  a[2] = byte(ID);
  a[3] = 0x05;    // length
  a[4] = 0x03;
  a[5] = 0x1E;
  a[6] = byte(pos);
  a[7] = byte((pos & 0xff00) >> 8);

  unsigned int sum = 0;
  int i;
  for(i = 2; i < 8; i++)
  {
    sum += a[i];
  }
  sum = ~byte(sum & 0x00FF);
  a[8] = sum;

  digitalWrite(controlpin, HIGH);
  
  for(int ii = 0; ii < 9; ii++)
  {
    USART_Transmit_for_1(a[ii]);
    //Serial.println(a[ii]);
  }
  delayMicroseconds(50);
  digitalWrite(controlpin, LOW);
}

void bulkRead()
{
  a[0] = 0xFF;
  a[1] = 0xFF;
  a[2] = 0xFE;
  a[3] = 0x06;   // length
  a[4] = 0x92;   
  a[5] = 0x00;
  a[6] = 0x02;   // first module data length
  a[7] = 0x01;   // first module id
  a[8] = 0x24;   // 현재 위치

  unsigned int sum = 0;
  for(int i = 2; i < 9; i++)
  {
    sum += a[i];
  }
  sum = ~byte(sum);
  a[9] = sum;
  for(int ii = 0; ii < 10; ii++)
  {
    USART_Transmit_for_1(a[ii]);
  }
}

ISR(TIMER1_COMPA_vect)
{
  ISR_cnt++;
  if(ISR_cnt % 5 == 1)
  {
    // TODO: 여기에 모터 write 코드 넣어주어야
    //motor_goal = dynamixel_CF_movement(MMDEGREE, MMFRE, ISR_cnt, pastDEGREE);
    //writeMotor(MIDc, motor_goal);
  }
  else if(ISR_cnt % 5 == 3)
  {
    // TODO: 모터 read?
  }
  // 50ms마다 한번 씩 매트랩으로 값을 쏴준다
  else if(ISR_cnt % 50 == 4)
  {
    send_data_to_matlab = true;
  }
  
  // TODO: understand completely
  if(Tcnt < Mcount)   // ISR -> 1ms마다 호출 => goal을 반주기로 나눠서 여러번 보냄?
  {
    Tcnt++;
    if((MMDEGREE - pastDEGREE) > 0)
    {
      w = pastDEGREE + (MMDEGREE - pastDEGREE)/2.0*(1-cos((2*3.14/(MMFRE*10.0))*Tcnt)); 
    }
    else
    { 
      w = pastDEGREE - (-1*(MMDEGREE - pastDEGREE))/2.0*(1-cos((2*3.14/(MMFRE*10.0))*Tcnt));
    }
    writeMotor(MIDc, w);
  }
  else
  {
    pastDEGREE = w;
    Serial.flush();
  }
 
}

void blinkLed()
{
  digitalWrite(ledpin, HIGH);
  delay(1000);
  digitalWrite(ledpin, LOW);
  delay(1000); 
}

void data_reading_from_matlab()
{
  int i;
  int MID[20];
  int MDEGRE0E;
  int MFRE;

  if(Serial.available() > 0)
  {
    char ID1 = Serial.read();
    int ID = ID1 - '0';

    if(ID == 49)
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
      MIDc = (MID[0]*10) + (MID[1]*1);
    }
    else if(ID == 50)
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
      MMDEGREE = (MID[0]*1000) + (MID[1]*100) + (MID[2]*10) + (MID[3]*1);
      Tcnt = 0;
      Mcount = 5 * MMFRE;   // 1 - cos의 반주기
    }
    else if(ID == 51)
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
      MFRE = (MID[0]*1000) + (MID[1]*100) + (MID[2]*10) + (MID[3]*1);
      MMFRE = MFRE;
      Mcount = 5 * MMFRE;   // 1 - cos의 반주기
    }
    if(MIDc == 1 && MMDEGREE == 1000 && MFRE == 1000)
    {
      blinkLed();
    }  
  }
}

void data_sending_to_matlab()
{
  //Serial.println(readpacket[1]);
  //Serial.println(readpacket[2]);
  send_data_to_matlab = false;
}

void loop() 
{
  // put your main code here, to run repeatedly:
  data_reading_from_matlab();

  if(send_data_to_matlab == true)
  {
    data_sending_to_matlab();
  }
}
