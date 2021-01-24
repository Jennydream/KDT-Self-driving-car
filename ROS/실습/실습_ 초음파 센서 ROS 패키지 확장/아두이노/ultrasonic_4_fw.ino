/*
HC-SR04 초음파 센서
*/

int trig[4]={2,4,6,8}; //트리거 핀
int echo[4]={3,5,7,9}; //에코 핀
long distance[4];  //거리 담기

void setup() {

   Serial.begin(9600);     // 통신속도 9600bps로 시리얼 통신 시작

   //4개 모두 선언해주기
   for(int i=0; i<4; i++){
    pinMode(trig[i], OUTPUT);  // 트리거 핀을 출력으로 선언
    pinMode(echo[i], INPUT);   // 에코핀을 입력으로 선언
  }
}

void connect_pin(){
  long duration;
  
  //4개 모두 선언해주기
  for(int i=0; i<4; i++){
    digitalWrite(trig[i], LOW);  // Trig 핀 Low
    delayMicroseconds(2);     // 2us 딜레이
    digitalWrite(trig[i], HIGH); // Trig 핀 High
    delayMicroseconds(10);    // 10us 딜레이
    digitalWrite(trig[i], LOW);  // Trig 핀 Low

  // pulseln() 함수는 핀에서 펄스신호를 읽어서 마이크로초 단위로 반환
    duration = pulseIn(echo[i], HIGH);
    distance[i] = duration * 170 / 1000; // 왕복시간이므로 340m를 2로 나누어 170 곱하
    }
    
  }

void loop() {
  
  //한번 회전 할 때마다 값 얻어오기
  connect_pin();

  // 각 거리를 시리얼 모니터에 출력
  Serial.print(distance[0]); 
  Serial.print("mm ");
  Serial.print(distance[1]); 
  Serial.print("mm ");
  Serial.print(distance[2]); 
  Serial.print("mm ");
  Serial.print(distance[3]);
  Serial.println("mm ");
  delay(100);
}
