/*
 RADL UWB system _ by EY
 ver 2:2
 <Initiator 3>
 */
 
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
char POLL = 0;
char POLL_ACK = 1;
char RANGE =  2;
char RANGE_REPORT= 3;
char RANGE_FAILED= 255;

#define UWB_CNT 2

char uwb_select = 2;
char uwb_status = 0;
double range_dist = 0;

//enum POLLSET {POLL POLL_ACK RANGE RANGE_REPORT RANGE_FAILED}

// message flow state
volatile byte expectedMsgId = POLL;
volatile byte cur_uwb = SELECT_POLL_A;

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

//user edit -> buffer set up
bool data_received = false;

/* 

*/
void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    // Debug set timer 
    delay(1000);
    Serial.println(F("###Anchor###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    //Serial.println(F("DW1000Ng initialized ..."));
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  	DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

    DW1000Ng::setDeviceAddress(1); // device set up for each data set : send data to the rx buff.
	
    DW1000Ng::setAntennaDelay(16359);
    
    //Serial.println(F("Committed configuration ..."));
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
    //receiver();
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

void transmitPoll_Select(int select_poll) {
    int set_uwb = 0;

    data[LEN_DATA-1] = set_uwb;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitPollAck(int select_poll) {
    int set_uwb = 0;
    data[0] = POLL_ACK;
    data[LEN_DATA-1] = set_uwb;

    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeReport(float curRange, int select_poll) {
    int set_uwb = 0;
    data[0] = RANGE_REPORT;
    data[LEN_DATA-1] = set_uwb;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void transmitRangeFailed(int select_poll) {
      int set_uwb = 0;
    data[0] = RANGE_FAILED;
    data[LEN_DATA-1] = set_uwb;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
}

void loop() {
    // select mode
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
            timePollAckSent = DW1000Ng::getTransmitTimestamp();
            noteActivity();
        }
        DW1000Ng::startReceive();
    }
    if (receivedAck) {
        receivedAck = false;
        // get message and parse

        DW1000Ng::getReceivedData(data, LEN_DATA);
  
        if(uwb_select == 1){
          if(data[LEN_DATA-1] != SELECT_POLL_A){
            DW1000Ng::startReceive();
            noteActivity();
            return;
          } 
        }
        else if(uwb_select == 2){
          if(data[LEN_DATA-1] != SELECT_POLL_B){
            DW1000Ng::startReceive();
            noteActivity();
            return;
          } 
        }
  
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
            transmitPollAck(uwb_select);
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
                /* Apply simple bias correction */
                distance = DW1000NgRanging::correctRange(distance); // cm단위로 변경
                range_dist = distance;
                short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex
                
                short power = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산 
                byte txbuf[8]={0xFF,0xFF,uwbdata[0],uwbdata[1],uwbdata[2],uwbdata[3],0xFF,0xFE}; //stx, data --- data , etx//

                uwbdata[0] = (dist >> 8) & 0xFF; 
                uwbdata[1] = dist & 0xFF;
                uwbdata[2] = (power >> 8) & 0xFF; 
                uwbdata[3] = power & 0xFF;
                //Serial.write(txbuf,sizeof(txbuf));    

              // update sampling rate (each second)
                transmitRangeReport(distance * DISTANCE_OF_RADIO_INV,uwb_select);
                successRangingCount++;
                if (curMillis - rangingCountPeriod > 1000) {
                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                    rangingCountPeriod = curMillis;
                    successRangingCount = 0;
                }           

            }
            else {
              
                transmitRangeFailed(uwb_select);
            }
                if(uwb_select == 1) uwb_select = 2;
                else if (uwb_select == 2) uwb_select = 1;
                uwb_status = 0;
                noteActivity();
            }
                if(uwb_status == 0){
                  double dist_uwb_a = 0;
                  double dist_uwb_b = 0;
                  if(cur_uwb == SELECT_POLL_A){
                  dist_uwb_a= range_dist;
                  }
                  else if(cur_uwb == SELECT_POLL_B){
                  dist_uwb_b = range_dist;
                  }
                  String rangeString = "RangeA: "; 
                  String rangeStringB = "RangeB: "; 
                  rangeString += dist_uwb_a;
                  rangeString += rangeStringB;
                  rangeString += dist_uwb_b;
                  Serial.println(rangeString);
                }
    }
}
