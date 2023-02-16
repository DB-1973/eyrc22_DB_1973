int IR1 = A4;
int IR2 = A3;
int IR3 = A2;
int IR4 = A1;
int IR5 = A0;

int Min = 0;
int Max = 1024;

int input[5]; 
int error = 0;
int previous_error = 0;
int d = 0;
int i = 0;
float Krp = 0;
float Krd = 0;
float Kri = 0;

void printList(){
    for(int i = 0; i < 5; i++){
      Serial.print(input[i]);
      Serial.print(" ");
    }
    Serial.println("");
}

void normalise(){
  for(int i = 0; i < 5; i++){
    input[i] = (input[i]-Min)/(Max-Min);
  }
}

void setup(){
    Serial.begin(9600);  
}
void loop(){
  input[0] = analogRead(IR1);
  input[1] = analogRead(IR2);
  input[2] = analogRead(IR3);
  input[3] = analogRead(IR4);
  input[4] = analogRead(IR5);
  //printList(); 
  error = ((-2 * input[0]) + (-1*input[1]) + (input[3]) + (2*input[4]));
  d = error - previous_error;
  i = error + previous_error;
  int velocity = Krp*(error) + Krd*d + Kri*i;
  printList();
  Serial.println(error); 
}
