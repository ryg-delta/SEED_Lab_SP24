#include <Wire.h>
#define MY_ADDR 8

volatile uint8_t incoming_msg[32] = {0};
volatile uint8_t msg_length = 0;
volatile uint8_t offset = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(MY_ADDR);
  Wire.onReceive(receive);

}

void loop() {
  if (msg_length > 0){
    printReceived();
    msg_length = 0;
  }

}


void printReceived(){
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("The message received is: ");
  for (uint8_t i = 0; i < msg_length; i++){
    char converted_val = incoming_msg[i];
    Serial.print(converted_val);
  }
  Serial.println("");
  Serial.print("The respective ASCII values are: ");
  for (uint8_t i = 0; i < msg_length; i++){
    Serial.print(String(incoming_msg[i])+" ");
  }
  Serial.println("");
}

void receive(){
  offset = Wire.read();
  while (Wire.available()){
    incoming_msg[msg_length] = Wire.read();
    msg_length++;
  }
}