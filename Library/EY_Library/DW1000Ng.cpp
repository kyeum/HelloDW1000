
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace DW1000EY {
	/*	
	void resetInactive() {
		// tag sends POLL and listens for POLL_ACK
		expectedMsgId = POLL_ACK;
		DW1000Ng::forceTRxOff();
		transmitPoll();
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
	*/

}