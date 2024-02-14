-------------------Assignment 1b------------------
Resources used:
https://pypi.org/project/smbus2/
https://learn.adafruit.com/adafruit-16x2-character-lcd-plus-keypad-for-raspberry-pi/python-usage

--------
Code - Python

#Silje Ostrem
#I2C test

#The goal of this code is to send an integer to the arduino
#From the raspberry Pi, and have the arduino send it back
#With 100 added (int+100)

from smbus2 import SMBus


import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#Change LCD to teal
lcd.color = [0, 100, 100]

#Start with initalizing the arduino and the number
arduino_addr = 0x08
numb = 1

#Prompt user
print("enter a number between 0 and 100")

#initialize arduino i2c
i2c = SMBus(1)
time.sleep(1)
data = 0

#While loop for integer sending
while(True):
    #get an integer
    ledstate = int(input(">>>>: "))
    #Write that integer to the arduino
    i2c.write_byte_data(arduino_addr, 0x01,ledstate)
    #Read that data from the arduino
    data = i2c.read_byte(arduino_addr)
    #Clear the LCD Screen
    lcd.clear()
    #Write to LCD
    data_str = f"The Value is \n {data}"
    lcd.message = data_str


-------code - Arduino C
//Silje Ostrem
//EENG 350

//The goal of this code is
//To respond to RPi with the int +100
//Using i2c
#include <Wire.h>
#define MY_ADDR 8
#define PI_ADDR 0x99
// Global variables to be used for I2C communication
volatile uint8_t offset = 0;
//Initialize variables used during I2C
volatile uint8_t msg = 0;
volatile uint8_t num = 0;
volatile uint8_t newNum = 0;

//Function to recieve data during ISR
void receive() {
// Set the offset, this will always be the first byte.
  offset = Wire.read();
  Serial.print(offset);
  Serial.print(" ");
  //read byte
  num = Wire.read();

}
//Function for .OnRequest ISR
void sendData() {
  //Add 100 and send it back
  newNum = num +100 ;
  Serial.print(newNum);
  Serial.print( "\n");
  Wire.write(newNum);
}
// Intialize ISRs and serial communication
void setup() {
Serial.begin(115200);
// We want to control the built-in LED (pin 13)
pinMode(LED_BUILTIN, OUTPUT);
// Initialize I2C
Wire.begin(MY_ADDR);
// Set callbacks for I2C interrupts
Wire.onReceive(receive);
Wire.onRequest(sendData);
}
void loop() {
//Do nothing except wait for ISR

  }


--------------------Assignment 2b ---------------------
Resources Used:
https://raspberrypi-guide.github.io/electronics/using-usb-webcams
https://docs.opencv.org/4.1.0/dc/d4d/tutorial_py_table_of_contents_gui.html
https://docs.opencv.org/3.4/df/d9d/tutorial_py_colorspaces.html
https://docs.opencv.org/3.1.0/d9/d61/tutorial_py_morphological_ops.html#gsc.tab=0

Questions:
(a) The different morphological transformations are: dilate, erode, open, close, gradient
top hat, black hat, and any combo of those
(b) It removes the boundaries of an object, which could help clarify edges
(c) Dilation does the opposite, it enlarges the boundaries, which will make the object seem larger

-----code
#Silje Ostrem
#The goal of this assigmnet
#is to take an image using the raspberry pi
#and then process it
import numpy as np
from time import sleep
import cv2 as cv

#This function takes a picture when called
def takePic():
    camera = cv.VideoCapture(0)
    # Let the camera warmup
    sleep(.5)
    # Get an image from the camera stream
    ret, image = camera.read()
    if not ret:
        print("Could not capture image from camera!")
        quit()
    else:
        return (image)

#Dilate the image
def dilateIM(image, kernel):
    return (cv.dilate(image, kernel, iterations=1))

#Erode the image
def erodeIM(image, kernel):
    return (cv.erode(image, kernel, iterations=1))

#Open the image
def openIM(image, kernel):
    return (cv.morphologyEx(image, cv.MORPH_OPEN, kernel))

#Close the image
def closeIM(image, kernel):
    return (cv.morphologyEx(image, cv.MORPH_CLOSE, kernel))

#Gradient tranformation
def gradeIM(image, kernel):
    return (cv.morphologyEx(image, cv.MORPH_GRADIENT, kernel))

#Top Hat transformation
def topIM(image, kernel):
    return (cv.morphologyEx(image, cv.MORPH_TOPHAT, kernel))

#Black hat transformation
def blackIM(image, kernel):
    return (cv.morphologyEx(image, cv.MORPH_BLACKHAT, kernel))
#Display the image
def dispIM(image, type):
    cv.imshow(f" {type} Image - press 0 to exit", image)
    cv.waitKey(0)
    cv.destroyAllWindows()



#Call pic function
colors= takePic()
#Turn into HSV
colorHSV = cv.cvtColor(colors,cv.COLOR_BGR2HSV)
#Display unaltered image
dispIM(colors, "basic image")
#Set Kernel
kernel = np.ones((5,5),np.uint8)
#Set upper and lower color limits
uppergreen =  np.array([0x60,0x255,0x60])
lowergreen = np.array([0,0x75,0])
#Set color mask from limits
mask = cv.inRange(colorHSV,lowergreen,uppergreen)
#Apply mask to image using bitwise and
result = cv.bitwise_and(colors,colors, mask=mask)
#Call disp function & display image
dispIM(result,"masked image")
#Dilate and disp image
dilatedIM = dilateIM(result,kernel)
dispIM(dilatedIM, "dilated image")
#Erode and disp image
erodedIM = erodeIM(result,kernel)
dispIM(erodedIM, "eroded image")
#Top hat transform
tophatIM =topIM(result,kernel)
dispIM(tophatIM, "Top Hat & Black Hat")
#Relase camera
camera.release()