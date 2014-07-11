/* Pkt - Logical Link Control layer for nRF24L01+ radio communications
 *
 * Based on packet_processor.c concept code used in Grill Monitor and other msprf24-based
 * applications.
 */


#ifndef PKT_H
#define PKT_H

#include <Arduino.h>
#include <Enrf24.h>


#define PKT_MAX_PKTLEN 16
#define PKT_DEFAULT_QUEUE_DEPTH 3
#define PKT_DEFAULT_PROGID_REGISTRATIONS 4

typedef void(*PKT_CALLBACK)(const uint8_t progID, const int len, const void *buffer);

typedef struct {
	uint8_t progID;
	uint8_t rfaddr[5];
	uint8_t pktlen;
	uint8_t buffer[PKT_MAX_PKTLEN];
} PktTXQueue;

typedef struct {
	uint8_t progID;
	PKT_CALLBACK callback;
} PktRXprogram;

class Pkt {
	private:
		Enrf24 *xcvr;
		PktTXQueue **txQueue;
		void *txQueueArrayBacking;
		unsigned int txQueueDepth;

		PktRXprogram **progRegs;
		unsigned int progRegMaxCount;
		void *progRegsArrayBacking;

		boolean do_deepsleep_after_tx;
		PKT_CALLBACK unknownProgramCallback;

	public:
		Pkt(Enrf24 *);
		void setTransceiver(Enrf24 *transceiver);
		void setTXqueueDepth(unsigned int queuedepth);
		void setMaxPrograms(unsigned int maxprogs);
		void begin();
		void end();

		// TX methods
		boolean send(const uint8_t progID, const uint8_t *rfaddr, const int len, const void *buffer);
		void flush();
		void setModeTXonly(boolean yesno);

		// RX methods
		boolean available();
		void loop();

		boolean attachProgram(const uint8_t progID, PKT_CALLBACK callback);
		boolean detachProgram(const uint8_t progID);

		boolean attachUnknownProgram(PKT_CALLBACK callback);
		boolean detachUnknownProgram();
};


#endif /* PKT_H */
