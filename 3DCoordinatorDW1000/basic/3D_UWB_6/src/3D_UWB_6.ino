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
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

#define POLL1 0
#define POLL2 4
#define POLL3 5
#define POLL4 6
#define POLL5 7
#define POLL6 8

#define RANGE_R 1
#define CHANGELEG 15


// message flow state
volatile byte expectedMsgId = POLL1;
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
uint8_t uwbdata[6] = {1,2,3,4,5,6};

int uwb_num = 0; // uwb numbering
short range1 = 0;
short range2 = 0;
short range3 = 0;
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
    expectedMsgId = POLL1;
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

void transmitPollAck() {
    data[0] = POLL_ACK;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
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

void transmitChangePoll() {
    data[0] = CHANGELEG;
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
 
void Send2PC(short dist, short power) {
                byte txbuf[8]={0xFF,0xFF,uwbdata[0],uwbdata[1],uwbdata[2],uwbdata[3],0xFF,0xFE}; //stx, data --- data , etx//
                uwbdata[0] = (dist >> 8) & 0xFF; 
                uwbdata[1] = dist & 0xFF;
                uwbdata[2] = (power >> 8) & 0xFF; 
                uwbdata[3] = power & 0xFF;
                Serial.write(txbuf,sizeof(txbuf));    
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
        if (msgId == RANGE) {

        timePollSent = DW1000Ng::getTransmitTimestamp();
        timeRangeReceived = DW1000Ng::getReceiveTimestamp();
        timePollReceived = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
        timeRangeSent = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);

        // (re-)compute range as two-way ranging is done
       /* double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);

        distance = DW1000NgRanging::correctRange(distance); // cm단위로 변경
       */
        double distance;
        //distance = ((timeRangeReceived - timeRangeSent) + (timePollReceived - timePollSent))/2;
       // distance = distance * DISTANCE_OF_RADIO;


        distance = DW1000NgRanging::computeRangeAsymmetric_2by2_EY(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
        distance = DW1000NgRanging::correctRange(distance); 




        //short power = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산
        //short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex           
       
       if(distance> 100) distance = 1;
       short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex           

       if(uwb_num==0){
           range1 = dist;
       }
       else if(uwb_num==1){
           range2 = dist;
       }
       else if(uwb_num==2){
           range3 = dist;
       }

        byte txbuf[10]={0xFF,0xFF,uwbdata[0],uwbdata[1],uwbdata[2],uwbdata[3],uwbdata[4],uwbdata[5],0xFF,0xFE}; //stx, data --- data , etx//
        uwbdata[0] = (range1 >> 8) & 0xFF; 
        uwbdata[1] = range1 & 0xFF;
        uwbdata[2] = (range2 >> 8) & 0xFF; 
        uwbdata[3] = range2 & 0xFF;
        uwbdata[4] = (range3 >> 8) & 0xFF; 
        uwbdata[5] = range3 & 0xFF;     


        Serial.write(txbuf,sizeof(txbuf));    
       
        String rangeString = "Range1: "; rangeString += range1;
        rangeString += "Range2: "; rangeString += range2;
        rangeString += "Range3: "; rangeString += range3;
        //Serial.println(rangeString);

        uwb_num++;
        uwb_num%=3; // total uwb numbering
        transmitPoll(uwb_num); // 1
        noteActivity();
        //checkFreq(curMillis);
        }
        else if(msgId == POLL6){
           // Serial.println("received poll;");
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


/* all system design*/
/*


#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <math.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

#define POLL2 4
#define POLL3 5
#define POLL4 6
#define POLL5 7
#define POLL6 8

#define RANGE_R 9
#define CHANGELEG 15

// message flow state
volatile byte expectedMsgId = POLL;
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
uint32_t resetPeriod = 100;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

//user defined data
uint8_t uwbdata[4] = {1,2,3,4};

int uwb_num = 0; // uwb numbering
double range1 = 0;
double range2 = 0;
double range3 = 0;
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
    DW1000Ng::setAntennaDelay(16530);
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
    expectedMsgId = POLL;
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

void transmitPollAck() {
    data[0] = POLL_ACK;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitPoll(int uwb_num) {
    if(uwb_num == 0){
        data[0] = POLL;
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

void transmitChangePoll() {
    data[0] = CHANGELEG;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
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
 
void Send2PC(short dist, short power) {
                byte txbuf[8]={0xFF,0xFF,uwbdata[0],uwbdata[1],uwbdata[2],uwbdata[3],0xFF,0xFE}; //stx, data --- data , etx//
                uwbdata[0] = (dist >> 8) & 0xFF; 
                uwbdata[1] = dist & 0xFF;
                uwbdata[2] = (power >> 8) & 0xFF; 
                uwbdata[3] = power & 0xFF;
                Serial.write(txbuf,sizeof(txbuf));    
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
                //Serial.println(F("0"));
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
        if (msgId == RANGE) {
        timePollSent = DW1000Ng::getTransmitTimestamp();
        timeRangeReceived = DW1000Ng::getReceiveTimestamp();
        timePollReceived = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
        timeRangeSent = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);

        // (re-)compute range as two-way ranging is done
        double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);

        distance = DW1000NgRanging::correctRange(distance); // cm단위로 변경
       
        double distance;
        distance = ((timeRangeReceived - timeRangeSent) + (timePollReceived - timePollSent))/2;
        distance = distance * DISTANCE_OF_RADIO;
        distance = DW1000NgRanging::correctRange(distance); 
        //short power = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산
        //short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex           
       if(uwb_num==0){
           range1 = distance;
       }
       else if(uwb_num==1){
           range2 = distance;
       }
       else if(uwb_num==2){
           range3 = distance;
       }
       
        String rangeString = "Range1: "; rangeString += range1;
        rangeString += "Range2: "; rangeString += range2;
        rangeString += "Range3: "; rangeString += range3;

        //rangeString += "\t Sampling: "; rangeString += samplingRate; rangeString += " Hz";
        Serial.println(rangeString);
        uwb_num++;
        uwb_num%=3; // total uwb numbering
        range1 = 0;range2 = 0;range3 = 0;
        transmitPoll(uwb_num); // 1
        noteActivity();
        checkFreq(curMillis);
        
        changecnt ++;
        if(changecnt>3){
        changecnt = 0;  
        transmitChangePoll();
   
        }

        }
        else if(msgId == CHANGELEG){
            uwb_num = 0;
            transmitPoll(uwb_num);
            noteActivity();
            return; // goto new loop//        

        }
        else if(msgId != RANGE){
            DW1000Ng::startReceive();
            noteActivity();
            return; // goto new loop//        
        }
     }
}

*/
/*
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <math.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

// message flow state
volatile byte expectedMsgId = POLL;
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
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 50;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

//user defined data
uint8_t uwbdata[4] = {1,2,3,4};
uint32_t tmr_1ms = 0;

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
bool data_received = false;

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    // Debug set timer 
    
    delay(1000);
    Serial.println(F("###3DUWB_6###"));

    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
    DW1000Ng::setDeviceAddress(1); // device set up for each data set : send data to the rx buff.
    DW1000Ng::setAntennaDelay(16530);
    
    Serial.println(F("Committed configuration ..."));
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
   
    receiver();
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
    expectedMsgId = POLL;
    receiver();
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

void transmitPollAck() {
    data[0] = POLL_ACK;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
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

    if (!sentAck && !receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Ng::getTransmitTimestamp()
            noteActivity();
        }
        DW1000Ng::startReceive();
    }
    if (receivedAck) {
        receivedAck = false;
        // get message and parse
        DW1000Ng::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
        }
        if (msgId == POLL) {
            // on POLL we (re-)start, so no protocol failure
            protocolFailed = false;
            timePollReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = RANGE;
            transmitPollAck();
            noteActivity();
        }
        else if (msgId == RANGE) {
            //data_received = true;
            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = POLL;
            if (!protocolFailed) {
                timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
                timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
                timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
                // (re-)compute range as two-way ranging is done
                double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
                distance = DW1000NgRanging::correctRange(distance); // cm단위로 변경
                short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex
                short power = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산

                Send2PC(dist,power);
                String rangeString = "Range: "; rangeString += distance;
                Serial.println(rangeString);

              // update sampling rate (each second)
                transmitRangeReport(distance * DISTANCE_OF_RADIO_INV);
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }
            }
            else {
                transmitRangeFailed();
            }
            noteActivity();
        }
    }
}

*/