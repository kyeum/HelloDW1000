/*
 RADL UWB system _ by EY
 ver 2:2
 <anchor 4>
 // end data bit 으로 전체 데이터 확인 및 파악!
 uwb ID set!
11/12/13/14 -> last bit!
 */

#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <math.h>

/*Todolist _EY
  pin
  set protocol in 2:2
*/
// connection arduino pins setup
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
char POLL_main = 5;

// char POLL_ACK 7 --> used in more accurate program using double two ways control
char RANGE_From1 = 1;
char RANGE_From2 = 2;
char RANGE_REPORT = 254;
char RANGE_FAILED = 255;
char RESETPOLL = 200; // Iter received from UWB board (2) 
// messages used in the ranging protocol
double range_dist = 0;

// message flow state
volatile byte expectedMsgId = POLL_main;


// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

// protocol error state
boolean protocolFailed = false;

// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;

// last computed range/time
uint64_t timeComputedRange;

// data buffer
#define LEN_DATA 17
byte data[LEN_DATA];

// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 10;

// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 1000;

// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

//configuration
device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_64MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true,
    true,
    true,
    false,
    true
};

//***user defined data
uint32_t tmr_1ms = 0;
bool received_from1 = false;
bool received_from2 = false;
//user edit -> buffer set up
bool data_received = false;
byte uwbdata[4] = {0xff, 0xff, 0xff, 0xff};
short dist_from1 = 0;
short dist_from2 = 0;
short power_from1 = 0;
short power_from2 = 0;


void setup() {
    Serial.begin(115200);
    pinMode(A5, OUTPUT);
    delay(1000);
    Serial.println(F("###UWB4 - Initiator###"));
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
    DW1000Ng::setDeviceAddress(4); 
    // device set up for each data set : send data to the rx buff.
    DW1000Ng::setAntennaDelay(16359);
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    //In Twoway, (4) would be initiator for the uwb comm system.
    //receiver();
    rangingCountPeriod = millis();
    }

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // anchor listens for POLL
    //expectedMsgId = POLL_main;
    //receiver();
    noteActivity();
}

void resetInactive_Initiator() {
    // anchor listens for POLL
    //expectedMsgId = POLL_main;
    //Initiator would send new transmit poll data in resetInactive
    transmitPoll();
    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitPoll() {
    data[0] = POLL_main;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}
void StartPoll() {
    Serial.println("Start poll");
    timePollSent = DW1000Ng::getTransmitTimestamp();
    received_from1 = false;
    received_from2 = false;
    transmitPoll();
    receiver();  
    noteActivity();
}
void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();

}

void UpdateRangeRate(int32_t curMills ){
                successRangingCount++;
                if (curMills - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMills - rangingCountPeriod);
                    rangingCountPeriod = curMills;
                    successRangingCount = 0;
                }       
  
}
void Send2PC(short dist, short power) {
                byte txbuf[8]={0xFF,0xFF,uwbdata[0],uwbdata[1],uwbdata[2],uwbdata[3],0xFF,0xFE}; //stx, data --- data , etx//
                uwbdata[0] = (dist >> 8) & 0xFF; 
                uwbdata[1] = dist & 0xFF;
                uwbdata[2] = (power >> 8) & 0xFF; 
                uwbdata[3] = power & 0xFF;
                Serial.write(txbuf,sizeof(txbuf));    
}

void loop() {
      int32_t curMillis = millis();
      //digitalWrite(A5, HIGH); check status with LED
 
        if (curMillis - lastActivity > 100) { // reset time rate!
        StartPoll();   
        }
        else{
        if(received_from1 && received_from2){
        received_from1 = false;
        received_from2 = false;
        Send2PC(dist_from1,power_from1);
        Send2PC(dist_from2,power_from2);
        }
        if (receivedAck) {
          receivedAck = false;
          DW1000Ng::getReceivedData(data, LEN_DATA);
          byte msgId = data[0];
          if(msgId == RESETPOLL){ // RESETPOLL
            StartPoll();   
          }
          else if (msgId == RANGE_From1) {
            //if (!protocolFailed) {}
              Serial.println("received range from 1");
              //protocolFailed = false;
              timeRangeReceived = DW1000Ng::getReceiveTimestamp();
              timeRangeSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
              
              double distance = DW1000EYRanging::computeRangeAsymmetric_2by2_EY(timePollSent,
                                                             timePollSent, 
                                                             timePollAckReceived, 
                                                            timePollAckReceived, 
                                                             timeRangeSent, 
                                                             timeRangeReceived);
              /* Apply simple bias correction */
              distance = DW1000NgRanging::correctRange(distance); // cm단위로 변경
              dist_from1 = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex
              power_from1 = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산 

              // update sampling rate (each second) -- for testing uwb
              UpdateRangeRate(curMillis);
              Serial.println(distance);
              received_from1 = true;
              noteActivity();
           }
          else if (msgId == RANGE_From2) {
              Serial.println("received range from 2");
              //protocolFailed = false;
              timeRangeReceived = DW1000Ng::getReceiveTimestamp();
              timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
              timeRangeSent = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
              
              double distance = DW1000EYRanging::computeRangeAsymmetric_2by2_EY(timePollSent,
                                                             timePollSent, 
                                                             timePollAckReceived, 
                                                            timePollAckReceived, 
                                                             timeRangeSent, 
                                                             timeRangeReceived);
              /* Apply simple bias correction */
              distance = DW1000EYRanging::correctRange(distance); 
              dist_from2 = (short)(distance * 100); // cm단위로 변경 
              power_from2 = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산 

              // update sampling rate (each second) -- for testing uwb
              UpdateRangeRate(curMillis);
              
              Serial.println(distance);
              received_from2 = true;
              
              noteActivity();
          }
          else{
             DW1000Ng::startReceive();
          } 
         }        
    }   
}

/*reference
 if(mode_datareceived_from1 == false || mode_datareceived_from2 == false){
       if (curMillis - lastActivity > 100) { // reset time rate!
          mode_datareceived_from1 = false;
          mode_datareceived_from2 = false;
          noteActivity();
          DW1000Ng::forceTRxOff();
          mode_test = true;
       }
 * /
 */
