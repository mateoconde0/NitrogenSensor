/*
 * Design implementation: 
 *  - Going to stay in a sleep state for an hour at a time (need a interrupt for hourly wakeup) 
 *  - Twice a day we are going to connect to the wifi and send the average to the database (can be done on an hourly interrupt and a counter) 
 *    - for wifi connectivity: need a die protocal to prevent wifi fail issues 
 *  - The rest is already implemented 
*/

//TODO: Find way to connect the ESP8266 library 


#include <AS726X.h>
//#include <ESP8266WiFi.h>

//important constants 
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024
#define TIMER_INTV 3600

void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();


bool isLEDOn = false;
int g_time = 0;


//sensor instance 
AS726X sensor;

//Sensor related variables
String Field1;
String Field2;
String Field3;
String HOST;
String API;

int BYTE;
String Violet;
String Violet2;
String Threshold;
String Threshold3;
String SEND;

const int NumOfVReading = 20;
int VReading[NumOfVReading];
int VIndex = 0;
int VTotal = 0;
int VAverage = 0;


float V;

int V2;

int thresh = 0;

byte GAIN = 2;
byte MEASUREMENT_MODE = 0;


//ESP related variables
const int esp_delay = 1000; // sets the delay between commands so the ESP8266 has time to respond since we are not reading the output of the ESP8622 with the feather
String api_key = "2EPCZQUEDYV6ZBH2"; // apikey for the channel  
String field1_value = "mjc049";
String transmission;
int transmissionLength = 0;
String inString = "";
#define ESP_RST 11
#define ESP_PWR 12 

volatile int reconAttpts = 0;

// statemachine state variable 
volatile int state = 0; 

// count of number of times data has been sent   
volatile int count = 0; 
// variable to store the average data 
volatile int avg = 0; 

//Timer interuppt stuff 
//const uint16_t t1_load = 0; 
//const uint16_t t1_comp = 16;//need to determine value 
// 
//create a timer for the simple timer library. This is plan b.
//SimpleTimer timer;

//int m_timer_id; 

void setup() {
   
  //open communication with the visible light sensor 
  Serial1.begin(115200);
  Serial.begin(115200);
  //initialize the sensor using the libraries starting function 
  //going to be communicating through I2c 
  Wire.begin();
  sensor.begin(Wire, GAIN, MEASUREMENT_MODE);
  
  //initializing arrays with zeros
  for (int thisVReading = 0; thisVReading < NumOfVReading; thisVReading++) {
     VReading[thisVReading] = 0;
  }
     
  //start timer for interrupt 
  startTimer(1);
  //ESP reset pin. Set low to reset 
//  pinMode(ESP_RST,OUTPUT);
  pinMode(ESP_PWR,OUTPUT);
  // Done on restart or startup 
  digitalWrite(ESP_PWR,HIGH);
  delay(1000);
  Serial1.println("AT+RST"); //Reset the device
  delay(1000);
  Serial1.println("AT+CWMODE=1");
  delay(1000);
  Serial1.println("AT+CWJAP=\"Rehema\",\"qazWSX11!!\""); //Setup WiFi and connect to wlan
  delay(8000);
  inString = Serial1.readString();
  Serial.print(inString);
  digitalWrite(ESP_PWR,LOW);  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(state == 2){
    //need to take the last measurement 
    takeMeasurement();
    // report to the database 
    reportData();
    Serial.println("State 2: ");
    Serial.println("Count: " + String(count));
    //set the count back to 0 
    count = 0; 
    state = 0; 
  }else if(state == 1){
    // measure and add to the average 
    Serial.println("State 1: ");
    Serial.println("Count: " + String(count));
    takeMeasurement();
    count++;
    state = 0; 
  }
  
  delay(10);
}
/*///______________________________________________________________________________________//////

  Timer interrupt functions 
  Adapted from: https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989

////______________________________________________________________________________________/////*/

void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  TcCount16* TC = (TcCount16*) TC3;
  
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  Serial.println(TC->COUNT.reg);
  Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3);
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}


void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    // Write callback here!!!
    if(g_time == TIMER_INTV){
      g_time = 0;
//      digitalWrite(LED_PIN, isLEDOn);
//      isLEDOn = !isLEDOn;
        if(count == 12){
          // report to data 
          state = 2; 
        }else if(count < 12){
          //take data 
          state = 1;
        }
    }else{
      g_time++;
    }
  }
}



/*///______________________________________________________________________________________//////

  Sensor Functions 

////______________________________________________________________________________________/////*/

void takeMeasurement(){
  //collect 10 data samples to do data smoothing processing and then send it to communication link
  //turn the light on 
  sensor.enableBulb(); 
  for (int i = 0; i < 20; i++) {
    sensor.takeMeasurements();
    //subtract the last reading from the total to keep sum of only last 20 values
    VTotal = VTotal - VReading[VIndex];
    V = sensor.getCalibratedViolet();
    V2 = (int) V;
    VReading[VIndex] = V2;
    //add to total to make total of last 10 values
    VTotal = VTotal + VReading[VIndex];
    //go to next element in array
    VIndex = VIndex + 1;
    //check if all 20 readings are done for smoothing
    if (VIndex >= NumOfVReading) {
      //go back to starting of array
      VIndex = 0;
    }
    //calculate the average:
    VAverage = VTotal / NumOfVReading;
    
    Violet = String(VAverage);
    
    delay(500); 
  }
  //add violet to the average 
  avg = avg + VAverage;
  //turn the light off 
  sensor.disableBulb();
}


/*///______________________________________________________________________________________//////

  WIFI Functions

////______________________________________________________________________________________/////*/


void reportData(){
  //Divide the average by 12 
  avg = avg/12; 
  bool success = false; 
  startConnection();
  success = logData();
  //put the wifi module to sleep 
  closeConnection();
}

/*
 * Opens the Connection between the ESP and thingspeak
 * */
void startConnection()
{
    digitalWrite(ESP_PWR,HIGH); 
    Serial1.println("AT+CIPMODE=0");
    delay(esp_delay);
    Serial1.print("AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",80\r\n");
    
    //do something to ensure connect if everything is OK then can log data.
    inString = Serial1.readString();
    Serial.println(inString);
    delay(esp_delay + 650);
    inString = inString.substring((inString.length() - 4),(inString.length() - 2));
    // if not ok then reconnect by reseting and then sending
    reconAttpts++;
    if (inString != "OK" && reconAttpts < 10)
    {
        Serial1.print("AT+RST\r\n");
        delay(5000);
        startConnection();
    }
    reconAttpts = 0;
    delay(esp_delay);
}

/*
 * Closes the connection between the ESP and thingspeak and puts the ESP into a sleep mode 
 */
void closeConnection(){
  //need to find a way to put the module into sleep mode 
//  Serial.println("Going to sleep");
  //sends the microprocessor to sleep until the reset pin is triggered. 
//  ESP.deepSleep(0);
  digitalWrite(ESP_PWR,LOW);
}

String get_threshold(int comp)
{
  if (comp < 350){
    thresh = 3;
  }
  else if (comp >= 350 && comp < 650){
    thresh = 2;
  }
  else {
    thresh = 1;
  }
  return String(thresh);
}

/*
 * Log the data to thingspeak. Opens connection by using startConnection;
 * */
bool logData()
{
    startConnection();
    delay(esp_delay);
    Field1 = "&field1=dgp014";
    Field2 = "&field2=";
    Field3 = "&field3=";
    
    API = "GET /update?api_key="+api_key;
    Threshold = get_threshold(avg);
    
    Violet2 = Field2 + String(avg);
    Threshold3 = Field3 + Threshold;

    HOST = " HTTP/1.1\r\nHost: api.thingspeak.com\r\n";
    SEND = API + Field1 + Violet2 + Threshold3 + HOST;

    Serial1.print("AT+CIPSEND=");
    BYTE = SEND.length() + 2;
    Serial1.println(BYTE);
    delay(10000);
    Serial1.println(SEND);
    delay(5000);
    inString = Serial1.readString();
    Serial.println(inString);
    inString = inString.substring((inString.length() - 4),(inString.length() - 2));
    reconAttpts++;
    if (inString != "ED" && inString != "OK" && reconAttpts < 10)
    {
        Serial1.print("AT+RST\r\n");
        delay(5000);
    }
    reconAttpts = 0;
    //Signifiy that we successfully transmitted the data 
    return true;
}
