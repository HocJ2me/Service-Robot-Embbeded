#define inA1 2 
#define inA2 3 
#define inB1 4
#define inB2 5 
#define hongngoai1 6
#define hongngoai2 7
#define hongngoai3 8
#define hongngoai4 9
#define ENA 10
#define ENB 11
int benphai;
int bentrai;
int giatrilech;



void setup() {
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(hongngoai1, INPUT);
  pinMode(hongngoai2, INPUT);
  pinMode(hongngoai3, INPUT);
  pinMode(hongngoai4, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(9600);
  analogWrite(ENA, 120);
  analogWrite(ENB, 120);//đi chậm cho khỏi trượt Line
}
void loop(){
  benphai = analogRead(hongngoai1)+analogRead(hongngoai2);
  bentrai = analogRead(hongngoai3)+analogRead(hongngoai4);
  if (benphai == 0 && bentrai == 0){
    lui();
  }//Trượt ra khỏi Line thì lùi lại
  else {
    giatrilech == benphai - bentrai;
  }
  if (giatrilech > 0){
    Serial.println("Lệch trái");
    quaytrai();//Lệch trái, thỳ quay trái
  }
  else if (giatrilech < 0){
    Serial.println("Lệch phải");
    quayphai();}//lệch phải thì quay phải
  else if (giatrilech == 0){
  dithang();}//Không lệch đi thẳng
}


void dithang(){
           digitalWrite(inA1,HIGH);
           digitalWrite(inA2,LOW);
           digitalWrite(inB1,HIGH);
           digitalWrite(inB2,LOW);
   
}
void lui(){
           digitalWrite(inA1,LOW);
           digitalWrite(inA2,HIGH);
           digitalWrite(inB1,LOW);
           digitalWrite(inB2,HIGH);
}
void quaytrai(){
           digitalWrite(inA1,HIGH);
           digitalWrite(inA2,LOW);
           digitalWrite(inB1,LOW);
           digitalWrite(inB2,LOW);
}
void quayphai(){
           digitalWrite(inA1,LOW);
           digitalWrite(inA2,LOW);
           digitalWrite(inB1,HIGH);
           digitalWrite(inB2,LOW);
}