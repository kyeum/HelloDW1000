/*
Todolist
(6) : (1) (2) (3) test
*/

#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgTime.hpp>
#include <math.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
//#define POLL 0
#define POLL_ACK 9
#define RANGE_L 10
#define RANGE_R 1

#define RANGE_REPORT 3
#define RANGE_FAILED 255

#define POLL1 0
#define POLL2 4
#define POLL3 5
#define POLL4 6
#define POLL5 7
#define POLL6 8

#define CHANGELEG 15


// message flow state
//volatile byte expectedMsgId = POLL1;
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
uint64_t timeComputedRange;

// last computed range/time
// data buffer
#define LEN_DATA 12
#define LEN_POLL 1

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

//user defined data
uint8_t uwbdata[12] = {1,2,3,4,5,6,7,8,9,10,11,12};

int uwb_num = 0; // uwb numbering
short range1 = 0;
short range2 = 0;
short range3 = 0;
short power1 = 0;
short power2 = 0;
short power3 = 0;
int changecnt = 0;


device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
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

//user edit -> buffer set up

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    // Debug set timer 
    delay(1000);
    Serial.println(F("###3DUWB_6###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
    DW1000Ng::setDeviceAddress(6); // device set up for each data set : send data to the rx buff.
    DW1000Ng::setAntennaDelay(16350);
    // DEBUG chip info and registers pretty printed
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
    // anchor starts in receiving mode, awaiting a ranging poll message
    
    delay(1000);
    transmitPoll(uwb_num); 
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = millis();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // anchor listens for POLL
   // expectedMsgId = POLL1;
    DW1000Ng::forceTRxOff();
    uwb_num++;
    uwb_num%=3; // total uwb numbering
    transmitPoll(uwb_num);
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

void transmitPoll(int uwb_num) {
    if(uwb_num == 0){
        data[0] = POLL1;
    }
    else if(uwb_num == 1){
       data[0] = POLL2;
    }
    else if(uwb_num == 2){
       data[0] = POLL3;
    }
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}


void transmitRange_Basic() {
    data[0] = RANGE_R;
    /* Calculation of future time */
    byte futureTimeBytes[LENGTH_TIMESTAMP];
	timeRangeSent = DW1000Ng::getSystemTimestamp();
	timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();
    DW1000NgUtils::writeValueToBytes(data + 1, timePollReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(data + 6, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

void transmitRangeReport(float curRange) {
    data[0] = RANGE_REPORT;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeFailed() {
    data[0] = RANGE_FAILED;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
}
 

void checkFreq(int cur_millis){
                  // update sampling rate (each second)
    successRangingCount++;
    if (cur_millis - rangingCountPeriod > 1000) {
    samplingRate = (1000.0f * successRangingCount) / (cur_millis - rangingCountPeriod);
    rangingCountPeriod = cur_millis;
    successRangingCount = 0;
    }
}

void loop() {
    int32_t curMillis = millis();  
    
    if (!sentAck&&!receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
           // Serial.println(F("0"));
            resetInactive();
        }
        return;
    }
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Ng::getTransmitTimestamp();
            noteActivity();
        }
        DW1000Ng::startReceive();
    }

    // MASTER BOARD : SENDING START SIGNAL - IN RESET PERIOD 
    if (receivedAck) {
        receivedAck = false;
        DW1000Ng::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId == RANGE_L) {

        timePollSent = DW1000Ng::getTransmitTimestamp();
        timeRangeReceived = DW1000Ng::getReceiveTimestamp();
        timePollReceived = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
        timeRangeSent = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);

        double distance;
        distance = DW1000NgRanging::computeRangeAsymmetric_2by2_EY(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
        distance = DW1000NgRanging::correctRange(distance); 

       if(distance> 100) distance = 1;
       short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex           
       short power = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산

       if(uwb_num==0){
           range1 = dist;
           power1 = power;
       }
       else if(uwb_num==1){
           range2 = dist;
           power2 = power;
       }
       else if(uwb_num==2){
           range3 = dist;
           power3 = power;
       }

        byte txbuf[16] = {0xFF,0xFF,uwbdata[0],uwbdata[1],uwbdata[2],uwbdata[3],uwbdata[4],uwbdata[5],
                        uwbdata[6],uwbdata[7],uwbdata[8],uwbdata[9],uwbdata[10],uwbdata[11],0xFF,0xFE}; //stx, data --- data , etx//
       
       
        uwbdata[0] = (range1 >> 8) & 0xFF; 
        uwbdata[1] = range1 & 0xFF;
        uwbdata[2] = (range2 >> 8) & 0xFF; 
        uwbdata[3] = range2 & 0xFF;
        uwbdata[4] = (range3 >> 8) & 0xFF; 
        uwbdata[5] = range3 & 0xFF;     

        uwbdata[6] = (power1 >> 8) & 0xFF; 
        uwbdata[7] = power1 & 0xFF;
        uwbdata[8] = (power2 >> 8) & 0xFF; 
        uwbdata[9] = power2 & 0xFF;
        uwbdata[10] = (power3 >> 8) & 0xFF; 
        uwbdata[11] = power3 & 0xFF;     


        Serial.write(txbuf,sizeof(txbuf));    
    
    
        uwb_num++;
        uwb_num%=3; // total uwb numbering
        transmitPoll(uwb_num); // 1
        noteActivity();
        //checkFreq(curMillis);
        }
        else if(msgId == POLL6){
            timePollReceived = DW1000Ng::getReceiveTimestamp();
            transmitRange_Basic();
            noteActivity();
        }
        else{
            DW1000Ng::startReceive();        
            return; // goto new loop//        
        }
     }
}

