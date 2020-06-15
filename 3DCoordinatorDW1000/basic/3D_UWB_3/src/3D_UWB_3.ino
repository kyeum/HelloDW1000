#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgConstants.hpp>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL1 0
#define POLL2 4
#define POLL3 5
#define POLL4 6
#define POLL5 7
#define POLL6 8

#define POLL_ACK 1
#define RANGE 2
#define RANGE_R 9
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define CHANGELEG 15

// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;
uint64_t timeComputedRange;
// data buffer
#define LEN_DATA 12
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
double range4 = 0;
double range5 = 0;
double range6 = 0;
int changecnt = 0;

bool sent_doublechecked = false;
bool receive_status = true; //define receive possibility!
//define change right angle
bool right_ang = false;


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

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("###3DUWB_3###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);\
    DW1000Ng::setNetworkId(3);
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
    // anchor starts by transmitting a POLL message
    receiver();
    noteActivity();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
    expectedMsgId = POLL_ACK;
    //DW1000Ng::forceTRxOff();
    //transmitPoll();
    receiver();
    noteActivity();
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    //startReceiver();
    DW1000Ng::startReceive();
}

void startReceiver(){
    //receive status : true ok 
    if(receive_status){
        receive_status = false;
        DW1000Ng::startReceive();
    }
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
    receive_status = true;
}

void transmitPoll(int uwb_num) {
    if(uwb_num == 0){
       data[0] = POLL4;
    }
    else if(uwb_num == 1){
       data[0] = POLL5;
    }
    DW1000Ng::setTransmitData(data, LEN_DATA);
                Serial.println(F("3"));

    DW1000Ng::startTransmit();
}
void transmitChangePoll() {
    data[0] = CHANGELEG;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRange_Basic() {
    data[0] = RANGE;
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
    if (!sentAck&&!receivedAck) {
        if (millis() - lastActivity > resetPeriod) {
            Serial.println(F("0"));
            resetInactive();
        }
        return;
    }
    if (sentAck) {
        if(sent_doublechecked){
            sent_doublechecked = false;
          //  transmitPoll(uwb_num);
            noteActivity();
            return;
        }
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Ng::getTransmitTimestamp();
            noteActivity();
        }
        else{
          //  startReceiver();
            DW1000Ng::startReceive();
        }
    }
    if  (receivedAck) {
        receivedAck = false;
        DW1000Ng::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId == POLL3) {
            Serial.println(F("1"));
            timePollReceived = DW1000Ng::getReceiveTimestamp();
            transmitRange_Basic();
            _delay_ms(100);
            transmitPoll(0);
            //sent_doublechecked = true;
            noteActivity();
            DW1000Ng::startReceive();

        }
        else if (msgId == RANGE_R) {

            timePollSent = DW1000Ng::getTransmitTimestamp();
            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
            timePollReceived = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
            timeRangeSent = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);

            double distance;
            distance = ((timeRangeReceived - timeRangeSent) + (timePollReceived - timePollSent))/2;
            distance = distance * DISTANCE_OF_RADIO;
            distance = DW1000NgRanging::correctRange(distance); 
            //short power = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산
            //short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex           
                if(uwb_num==0){
                range4 = distance;
                }
                else if(uwb_num==1){
                range5 = distance;
                }
            Serial.println("2");
            String rangeString = "Range1: "; rangeString += range4;
            rangeString += "Range2: "; rangeString += range5;

        //rangeString += "\t Sampling: "; rangeString += samplingRate; rangeString += " Hz";
        //Serial.println(rangeString);
        /*
        uwb_num++;
        uwb_num%=2; // total uwb numbering
        range4 = 0;range5 = 0;
        changecnt ++;
        if(changecnt>2){
        changecnt = 0;  
        }
        else{
        transmitPoll(uwb_num); // 1
        }
        */
        //transmitPoll(uwb_num); // 1
        
        noteActivity();
        }
        else{
//          startReceiver();
            DW1000Ng::startReceive();
            noteActivity();
            return; // goto new loop//
        }
    }    
}

/* all system */

/*

#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgConstants.hpp>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0
#define POLL2 4
#define POLL3 5
#define POLL4 6
#define POLL5 7
#define POLL6 8

#define POLL_ACK 1
#define RANGE 2
#define RANGE_R 9
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define CHANGELEG 15

// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
uint64_t timeRangeReceived;
uint64_t timeComputedRange;
// data buffer
#define LEN_DATA 12
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
double range4 = 0;
double range5 = 0;
double range6 = 0;
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

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("###3DUWB_3###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);\
    DW1000Ng::setNetworkId(3);
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
    // anchor starts by transmitting a POLL message
    receiver();
    noteActivity();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
    expectedMsgId = POLL_ACK;
    //DW1000Ng::forceTRxOff();
    //transmitPoll();
    receiver();
    noteActivity();
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
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
        data[0] = POLL4;
    }
    else if(uwb_num == 1){
       data[0] = POLL5;
    }
    else if(uwb_num == 2){
       data[0] = POLL6;
    }
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}
void transmitChangePoll() {
    data[0] = CHANGELEG;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}


void transmitRange_Doublesided() {
    data[0] = RANGE;
    byte futureTimeBytes[LENGTH_TIMESTAMP];
	timeRangeSent = DW1000Ng::getSystemTimestamp();
	timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();
    DW1000NgUtils::writeValueToBytes(data + 1, timePollSent, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(data + 6, timePollAckReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(data + 11, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
}


void transmitRange_Basic() {
    data[0] = RANGE;
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
    
    if (!sentAck&&!receivedAck) {
        // check if inactive
        if (millis() - lastActivity > resetPeriod) {
            resetInactive();
            Serial.println(F("0"));

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
    
    if (receivedAck) {
        receivedAck = false;
        DW1000Ng::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != POLL) {
            DW1000Ng::startReceive();
            noteActivity();
            return; // goto new loop//
        }
        else if (msgId == POLL) {
            Serial.println("received poll;");
            timePollReceived = DW1000Ng::getReceiveTimestamp();
            transmitRange_Basic();
            noteActivity();
        }
        else if (msgId == RANGE_R) {
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
           range4 = distance;
       }
       else if(uwb_num==1){
           range5 = distance;
       }
       else if(uwb_num==2){
           range6 = distance;
       }
       
        String rangeString = "Range1: "; rangeString += range4;
        rangeString += "Range2: "; rangeString += range5;
        rangeString += "Range3: "; rangeString += range6;

        //rangeString += "\t Sampling: "; rangeString += samplingRate; rangeString += " Hz";
        Serial.println(rangeString);
        uwb_num++;
        uwb_num%=3; // total uwb numbering
        range4 = 0;range5 = 0;range6 = 0;
        transmitPoll(uwb_num); // 1
        noteActivity();
        //checkFreq(curMillis);
      
        //cnt 3 -> change poll 
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
    }    
}
*/



/*
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

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
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
uint64_t timePollSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;
// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 50;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

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

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("###3DUWB_1###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    Serial.println("DW1000Ng initialized ...");
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);\
    DW1000Ng::setNetworkId(1);
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
    // anchor starts by transmitting a POLL message
    transmitPoll();
    noteActivity();
}

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
    expectedMsgId = POLL_ACK;
    DW1000Ng::forceTRxOff();
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
    data[0] = POLL;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRange() {
    data[0] = RANGE;
    byte futureTimeBytes[LENGTH_TIMESTAMP];
	timeRangeSent = DW1000Ng::getSystemTimestamp();
	timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();

    DW1000NgUtils::writeValueToBytes(data + 1, timePollSent, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(data + 6, timePollAckReceived, LENGTH_TIMESTAMP);
    DW1000NgUtils::writeValueToBytes(data + 11, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

void loop() {
     
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (millis() - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        DW1000Ng::startReceive();
    }
    if (receivedAck) {
        receivedAck = false;
        // get message and parse
        DW1000Ng::getReceivedData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again
            //Serial.print("Received wrong message # "); Serial.println(msgId);
            expectedMsgId = POLL_ACK;
            transmitPoll();
            return;
        }
        if (msgId == POLL_ACK) {
            timePollSent = DW1000Ng::getTransmitTimestamp();
            timePollAckReceived = DW1000Ng::getReceiveTimestamp();
            expectedMsgId = RANGE_REPORT;
            transmitRange();
            noteActivity();
        } else if (msgId == RANGE_REPORT) {
            expectedMsgId = POLL_ACK;
            float curRange;
            memcpy(&curRange, data + 1, 4);
            transmitPoll();
            noteActivity();
        } else if (msgId == RANGE_FAILED) {
            expectedMsgId = POLL_ACK;
            transmitPoll();
            noteActivity();
        }
    }
}
*/