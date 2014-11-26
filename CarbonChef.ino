#include <SD.h>
#include <PID_v1.h>

//MAX6675 Pins
#define MAX_CS 3               
#define MAX_SO 2               
#define MAX_SCK 4             

//SD Card
#define SD_CS 8
#define SD_MOSI 11
#define SD_MISO 12
#define SD_SCK 13

//Heater relay
#define heaterPin A5

//PID
#define Kp  10       // the Proportional control constant 10
#define Ki  0.5         // the Integral control constant 0.5
#define Kd  100         // the Derivative control constant 100

//Initialize the MAX6675 Library for our chip
const int units = 1;       // Units to readout temp (0 = ˚F, 1 = ˚C)

//Initiate PID
double current_temp, should_be, Output;  //Define Variables we'll be connecting to
PID oven_PID(&current_temp, &Output, &should_be, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowstart_time;

//For Digital Smooth
const int filter_samples = 5;              // filter_samples should  be an odd number, no smaller than 3

const int TARGET_TEMP = 0;
const int RAMP_RATE = 1;
const int DWELL_TIME_MINUTES = 2;

const int DONE = -1;

//Temperature profile
float steps[10][3] = {
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}, 
  {-1, -1, -1}
}; 

int index = 0;
float start_temp = 18;
float start_time = 0;
float target_reached_time = 0;
boolean running = false;
long total_time;

File profiles;
File logFile;

//Setup------------------------------------------------------
void setup(void) {

  Serial.begin(9600);

  //Pins
  pinMode(MAX_CS, OUTPUT);
  pinMode(MAX_SO, INPUT);
  pinMode(MAX_SCK, OUTPUT);
  digitalWrite(MAX_CS, HIGH);

  pinMode(heaterPin, OUTPUT);

  //PID
  oven_PID.SetOutputLimits(0, WindowSize);  //tell the PID to range between 0 and the full window size  
  oven_PID.SetMode(AUTOMATIC);  //turn the PID on

  pinMode(10, OUTPUT);

  if (!SD.begin(SD_CS)) {
    Serial.println("Failed to init SD");
    return;
  }

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  profiles = SD.open("profiles.csv");
  if (!profiles) {
    Serial.println("error opening profiles.csv");
    return;    
  }
  index = 0;
  while (profiles.available() && index < 10) {
    steps[index][TARGET_TEMP] = profiles.parseFloat();
    Serial.print("Target temp:"); Serial.println(target_temp());
    profiles.read(); //read the comma
    steps[index][RAMP_RATE] = profiles.parseFloat();
    Serial.print("Ramp ratep:"); Serial.println(ramp_rate());
    profiles.read(); //read the comma
    steps[index][DWELL_TIME_MINUTES] = profiles.parseFloat();
    Serial.print("Dwell time:"); Serial.println(dwell_time());
    profiles.read(); //read end of line
    index++;
  }
  profiles.close();

  index=0;

  // re-open the file for reading:
  logFile = SD.open("log.txt");
  if (!logFile) {
    Serial.println("error opening log.txt");
    return;
  }  
  start_oven_program();
  Serial.println("Oven Controller Initiated");
}

//Main Loop-------------------------------------------------------------------------------
void loop(){
  if (!running) return;
  total_time++;
  should_be = get_target_temperature();

  if (should_be == DONE) {
    stop_oven_program();
    return;
  }

  current_temp = read_temperature();
  if (should_turn_oven_on()){
    digitalWrite(heaterPin, HIGH);
  } else{
    digitalWrite(heaterPin, LOW);
  }

  boolean done_dwelling = dwelling() && (total_time - target_reached_time) > (dwell_time() * 60);
  boolean reached_target = current_temp >= steps[index][TARGET_TEMP];
  if (done_dwelling){
    index++;
    target_reached_time = 0;
    start_time = total_time;
    start_temp = current_temp;
  } else if (reached_target){ //switch to dwell time
    target_reached_time = total_time;
  } 
  log(total_time, current_temp, should_be);
    
  //TODO instead of delaying, perform the above logic only every 1000 milliseconds,
  //but check for changes in START/STOP-buttons every loop
  delay(1000);
}

void start_oven_program(){
  running = true;
  start_temp = read_temperature();
  digitalWrite(heaterPin, HIGH);    
  total_time = 0;
  start_time = 0;       
  target_reached_time = 0;  
}

void stop_oven_program() {
  logFile.close();
  running = false;
  digitalWrite(heaterPin, LOW);
  total_time = 0;       
  target_reached_time = 0;  
}

boolean should_turn_oven_on() {  
  oven_PID.Compute();
  unsigned long time_to_switch = (millis() - windowstart_time);
  if(time_to_switch > WindowSize) { //time to shift the Relay Window
    windowstart_time += WindowSize;
  }
  return (Output > time_to_switch);
}

boolean should_turn_oven_on_simple(){
  return (current_temp < (should_be - 0.5));
}

float get_target_temperature() {
  float target = steps[index][TARGET_TEMP];
  if (dwelling() || target == DONE) {
    return target;
  }
  int time_spent_this_step = total_time - start_time;
  return start_temp + ((steps[index][RAMP_RATE] / 60) * time_spent_this_step);
}

float dwell_time() {
  return steps[index][DWELL_TIME_MINUTES];
}

float ramp_rate(){
  return steps[index][RAMP_RATE];
}

float target_temp(){
  return steps[index][TARGET_TEMP];
}

boolean dwelling() {
  return target_reached_time > 0;
}

int compare (const void * a, const void * b){
  return (int)(*(float*)a - *(float*)b);
}

float read_temperature(){  

  float sort[filter_samples]; //temp0.read_temp(1);
  int current_pos = 0;
  int return_pos = ((filter_samples - 1) / 2) + 1;
  float temporary;
  int value = 0;
  float temp = 0;
  int error = 0; 

  for (int i=0; i < filter_samples; i++){
    digitalWrite(MAX_CS,LOW); // Enable device

    /* Cycle the clock for dummy bit 15 */
    digitalWrite(MAX_SCK,HIGH);
    digitalWrite(MAX_SCK,LOW);

    /* Read bits 14-3 from MAX6675 for the Temp 
     Loop for each bit reading the value and 
     storing the final value in 'temp' 
     */
    for (int i=11; i>=0; i--){
      digitalWrite(MAX_SCK,HIGH);  // Set Clock to HIGH
      value += digitalRead(MAX_SO) << i;  // Read data and add it to our variable
      digitalWrite(MAX_SCK,LOW);  // Set Clock to LOW
    }

    /* Read the TC current_temp inp to check for TC Errors */
    digitalWrite(MAX_SCK,HIGH); // Set Clock to HIGH
    error = digitalRead(MAX_SO); // Read data
    digitalWrite(MAX_SCK,LOW);  // Set Clock to LOW

    digitalWrite(MAX_CS, HIGH); //Disable Device

    if (error != 0) {
      value = -1;
    }

    sort[current_pos] = value;
    value = 0;
    current_pos++;
  }

  qsort(sort, filter_samples, sizeof(float), compare);
  return sort[return_pos] / 4;  //thermocouple has a resolution of .25 degrees
}


void log(long time, float temp, float target_temp){
  //enableSPI();
  logFile.print(time);
  logFile.print(",");
  logFile.print(temp,1);
  logFile.print(",");
  logFile.println(target_temp,1);
  
  // print to the serial port too:
   Serial.print(time);
   Serial.print(",");
   Serial.print(temp,1);
   Serial.print(",");
   Serial.println(target_temp,1);
}

