/*
 RADL UWB system _ by EY
 ver 2:2
 <anchor 4>
 */

#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <math.h>

// connection pins setup
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
char POLL = 5;
char POLL_ACK_from1 = 6;
char POLL_ACK_from2 = 7;

char RANGE =  2;
char RANGE_REPORT= 3;
char RANGE_FAILED= 255;
#define UWB_CNT 2

char uwb_select = 2;
char uwb_status = 0;
double range_dist = 0;

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
bool mode_transmit = true;
bool mode_test = true;
bool mode_datareceived_from1 = false;
bool mode_datareceived_from2 = false;



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

void setup() {
    Serial.begin(115200);
    pinMode(A5, OUTPUT);
    delay(1000);
    //set name 
    Serial.println(F("###Anchor4###"));
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
    // anchor starts in receiving mode, awaiting a ranging poll message
    //receiver();
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
    //data[0] = POLL_ACK;
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

void transmitPoll() {
    data[0] = POLL;
   //data[SELECT_POLL-1] = SELECT_POLL;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
}

void receiver() {
    DW1000Ng::forceTRxOff();
    // so we don't need to restart the receiver manually
    DW1000Ng::startReceive();
}

void loop() {
      int32_t curMillis = millis();
      digitalWrite(A5, HIGH);

     if(mode_datareceived_from1 == false || mode_datareceived_from2 == false){
       if (curMillis - lastActivity > 100) {
          mode_datareceived_from1 = false;
          mode_datareceived_from2 = false;
          noteActivity();
          DW1000Ng::forceTRxOff();
          mode_test = true;
       }
     }
     
    if(mode_test){
        for(int i =0; i<1; i++){
        timePollSent = DW1000Ng::getTransmitTimestamp();
        transmitPoll();
        Serial.println("sendpoll");
        mode_test = false;
        DW1000Ng::startReceive();
        noteActivity();
      }
   }
    else{     
    if (receivedAck) {
        receivedAck = false;
        DW1000Ng::getReceivedData(data, LEN_DATA);

        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again (except if already POLL)
            protocolFailed = true;
            
        }
        if (msgId == POLL_ACK_from1) {
            Serial.println("received range from 1");
            //protocolFailed = false;
            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
            timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
            timeRangeSent = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
            double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                           timePollSent, 
                                                           timePollAckReceived, 
                                                          timePollAckReceived, 
                                                           timeRangeSent, 
                                                           timeRangeReceived);

            Serial.println(distance);
            mode_datareceived_from1 = true;
            noteActivity();
        }
        if (msgId == POLL_ACK_from2) {
            Serial.println("received range from 2");
            //protocolFailed = false;
            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
            mode_datareceived_from2 = true;
            noteActivity();
        }
        if(mode_datareceived_from2&&mode_datareceived_from1)
        {                  
            mode_datareceived_from2 = false;
            mode_datareceived_from1 = false;
            mode_test = true;
        }
        else{
           DW1000Ng::startReceive();
        }

//        else if (msgId == RANGE) {
//            //data_received = true;
//            Serial.println("received range");
//
//            timeRangeReceived = DW1000Ng::getReceiveTimestamp();
//            expectedMsgId = POLL;
//            if (!protocolFailed) {
//                timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
//                timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
//                timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
//                // (re-)compute range as two-way ranging is done
//                double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
//                                                            timePollReceived, 
//                                                            timePollAckSent, 
//                                                            timePollAckReceived, 
//                                                            timeRangeSent, 
//                                                            timeRangeReceived);
//                /* Apply simple bias correction */
//                distance = DW1000NgRanging::correctRange(distance); // cm단위로 변경
//                range_dist = distance;
//                short dist = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex
//                
//                short power = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산 
//                byte txbuf[8]={0xFF,0xFF,uwbdata[0],uwbdata[1],uwbdata[2],uwbdata[3],0xFF,0xFE}; //stx, data --- data , etx//
//
//                uwbdata[0] = (dist >> 8) & 0xFF; 
//                uwbdata[1] = dist & 0xFF;
//                uwbdata[2] = (power >> 8) & 0xFF; 
//                uwbdata[3] = power & 0xFF;
//                //Serial.write(txbuf,sizeof(txbuf));    
//
//              // update sampling rate (each second)
//                transmitRangeReport(distance * DISTANCE_OF_RADIO_INV,uwb_select);
//                successRangingCount++;
//                if (curMillis - rangingCountPeriod > 1000) {
//                    samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
//                    rangingCountPeriod = curMillis;
//                    successRangingCount = 0;
//                }           
//
//            }
//            else {
//              
//                transmitRangeFailed(uwb_select);
//            }
//           }
      }  
    }
}