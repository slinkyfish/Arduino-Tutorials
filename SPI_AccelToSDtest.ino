//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <SD.h>

uint8_t CS = 10;
uint8_t CD = 8;
String dataString = "";
File dataFile;

int AccelCS = 9; //Different CS to SD card (unique)

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x, y, z;

void setup() {
  pinMode(10, OUTPUT);
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(AccelCS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(AccelCS, HIGH);

  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode

  // pinMode(CD, INPUT);
  //while (digitalRead(CD) == 0); //don't do anything until card is detected
  Serial.println("Card Inserted, Initializing SD card...");  //inform user of status
  SD.begin(CS);   //Start SD communication using CS pin
  Serial.println("Card initialized.");

}

void loop() {
  SPI.setDataMode(SPI_MODE3);
  delay(30);
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int)values[1] << 8) | (int)values[0];
  //The Y value is stored in values[2] and values[3].
  y = ((int)values[3] << 8) | (int)values[2];
  //The Z value is stored in values[4] and values[5].
  z = ((int)values[5] << 8) | (int)values[4];

  //Print the results to the terminal.
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.println(z, DEC);
  delay(10);

  SPI.setDataMode(SPI_MODE0);
  delay(30);
  int sensorValue = 9;
  dataString = "";
  dataString += String(x);  //convert int to string and append to existing string
  dataString += ", ";
  dataString += String(y);  //convert int to string and append to existing string
  dataString += ", ";
  dataString += String(z);  //convert int to string and append to existing string

  dataFile = SD.open("datalog.txt", FILE_WRITE);  //open file to write data to
  if (dataFile) {                 //If data file is actually open...
    dataFile.println(dataString); //save data
    dataFile.close();             //close file
    Serial.println(dataString);     // print to the serial port too:
  }
  else {
    Serial.println("error opening datalog.txt");    // if the file isn't open, pop up an error
  }

}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value) {
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(AccelCS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(AccelCS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values) {
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if (numBytes > 1)address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(AccelCS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for (int i = 0; i < numBytes; i++) {
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(AccelCS, HIGH);
}
