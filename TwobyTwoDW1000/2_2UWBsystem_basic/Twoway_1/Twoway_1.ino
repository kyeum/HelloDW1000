/*
 RADL UWB system _ by EY
 ver 2:2
 <Initiator 1>
 */

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
#define POLL 5
#define POLLtoThree 6 // 1 <-> 3 : 6, 2 <-> 3 : 7
#define POLLtoTwo 9 // 1 <-> 3 : 6, 2 <-> 3 : 7
#define RANGE 1
#define RANGE_REPORT 15
#define RANGE_FAILED 255
#define RANGE_From3 10



// message flow state
volatile byte expectedMsgId = POLL;

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

// timestamps to remember
uint64_t timePollSent_from3;
uint64_t timePollReceived_from3;
uint64_t timePollAckSent_from3;
uint64_t timePollAckReceived_from3;
uint64_t timeRangeSent_from3;
uint64_t timeRangeReceived_from3;

// data buffer
#define LEN_DATA 7
#define LEN_FULLDATA 16

byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 10;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 1000;

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

//************************************************user defined data
bool poll_received = false;
bool poll_send = false;
byte uwbdata[4] = {0xff, 0xff, 0xff, 0xff};
short dist_from3 = 0;
short power_from3 = 0;

void setup() {
    // DEBUG monitoring
    Serial.begin(115200);
    pinMode(A5, OUTPUT);
    delay(1000);
    
    Serial.println(F("###UWB1 - Responder###"));
    // initialize the driver
    DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
    // general configuration
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
    DW1000Ng::setNetworkId(1);
    DW1000Ng::setAntennaDelay(16359);
    
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

void transmitPoll() {
    data[0] = POLLtoThree;
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit();
    timePollSent_from3 = DW1000Ng::getTransmitTimestamp();
    noteActivity();
}

void transmitRange() {
    data[0] = RANGE;
    /* Calculation of future time */
    byte futureTimeBytes[LENGTH_TIMESTAMP];

    timeRangeSent = DW1000Ng::getSystemTimestamp();
    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
    DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
    DW1000Ng::setDelayedTRX(futureTimeBytes);
    timeRangeSent += DW1000Ng::getTxAntennaDelay();

    DW1000NgUtils::writeValueToBytes(data + 1, timeRangeSent, LENGTH_TIMESTAMP);//5byte 전송 : 총 6byte
    DW1000Ng::setTransmitData(data, LEN_DATA);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
}

void transmitDataToResponder(short dist, short power) {
    data[0] = POLLtoThree;
    DW1000NgUtils::writeValueToBytes(data + 1, timePollSent_from3, LENGTH_TIMESTAMP);//5byte 전송 : 총 6byte
    DW1000NgUtils::writeValueToBytes(data + 6, timeRangeSent_from3, LENGTH_TIMESTAMP);//5byte 전송 : 총 6byte
    DW1000NgUtils::writeValueToBytes(data + 11, timeRangeReceived_from3, LENGTH_TIMESTAMP);//5byte 전송 : 총 6byte

    DW1000Ng::setTransmitData(data, LEN_FULLDATA);
    DW1000Ng::startTransmit(TransmitMode::DELAYED);
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
    // wait for message before beginning// 
    int32_t curMillis = millis();
    if(poll_received){
          if(!poll_send){
            poll_send = true;
            transmitPoll();
          }
          if (curMillis - lastActivity > 100) { // reset time rate!
            transmitPoll(); //  POLL TRANSMIT until recogn 
          }
          if (receivedAck) {
          receivedAck = false;
  
          DW1000Ng::getReceivedData(data, LEN_DATA);
          byte msgId = data[0];
          if (msgId != RANGE_From3) {
              DW1000Ng::startReceive();
              return;
          }
          else if (msgId == RANGE_From3) {
              poll_received = true;
              Serial.println("received range from 3;");
              timeRangeReceived_from3 = DW1000Ng::getReceiveTimestamp();
              timeRangeSent_from3 = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
              double distance = DW1000EYRanging::computeRangeAsymmetric_2by2_EY(timePollSent,
                                                             timePollSent, 
                                                             timePollAckReceived, 
                                                            timePollAckReceived, 
                                                             timeRangeSent, 
                                                             timeRangeReceived);
              /* Apply simple bias correction */
              distance = DW1000NgRanging::correctRange(distance); // cm단위로 변경
              dist_from3 = (short)(distance * 100); // 아두이노 : 16비트 연산가능 //  convert short to hex
              power_from3 = (short)(DW1000Ng::getReceivePower()* 100); // 2byte 연산
              transmitDataToResponder(dist_from3, power_from3);
              noteActivity();
          }
        }     
    }
    else{
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
              poll_received = true;
              Serial.println("received poll;");
              timePollReceived = DW1000Ng::getReceiveTimestamp();
              transmitRange();
              noteActivity();
        } 
      }
    }
}
