/* Pkt - Logical Link Control layer for nRF24L01+ radio communications
 *
 * Based on packet_processor.c concept code used in Grill Monitor and other msprf24-based
 * applications.
 *
 *
 * Copyright (c) 2014, Eric Brundick <spirilis@linux.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose
 * with or without fee is hereby granted, provided that the above copyright notice
 * and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT,
 * OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <Arduino.h>
#include "Pkt.h"


/* Initialization, deinitialization and maintenance/utility functions */
Pkt::Pkt(Enrf24 *transceiver)
{
	unsigned int i;

	xcvr = transceiver;
	txQueueDepth = PKT_DEFAULT_QUEUE_DEPTH;
	progRegMaxCount = PKT_DEFAULT_PROGID_REGISTRATIONS;
	do_deepsleep_after_tx = false;

	txQueue = NULL;
	txQueueArrayBacking = NULL;
	progRegs = NULL;
	progRegsArrayBacking = NULL;
	unknownProgramCallback = NULL;
	defaultCallback = NULL;
}

void Pkt::setTransceiver(Enrf24 *transceiver)
{
	if (transceiver != NULL)
		xcvr = transceiver;
}

void Pkt::setTXqueueDepth(unsigned int queuedepth)
{
	unsigned int i;

	if (!queuedepth)
		return;

	if (txQueue != NULL) {  // should always be true
		free(txQueue);
		free(txQueueArrayBacking);
	}
	txQueueDepth = queuedepth;
	txQueue = (PktTXQueue **)malloc(sizeof(PktTXQueue *) * txQueueDepth);
	txQueueArrayBacking = malloc(sizeof(PktTXQueue) * txQueueDepth);

	for (i=0; i < txQueueDepth; i++) {
		txQueue[i] = (PktTXQueue *)((uint8_t *)txQueueArrayBacking + sizeof(PktTXQueue)*i);
		txQueue[i]->progID = 0xFF;
		txQueue[i]->pktlen = 0;
		memset(txQueue[i]->rfaddr, 0, 5);
		memset(txQueue[i]->buffer, 0, PKT_MAX_PKTLEN);
	}
}

void Pkt::setMaxPrograms(unsigned int maxprogs)
{
	unsigned int i;

	if (!maxprogs)
		return;

	if (progRegs != NULL) {  // should always be true
		free(progRegs);
		free(progRegsArrayBacking);
	}
	progRegMaxCount = maxprogs;
	progRegs = (PktRXprogram **)malloc(sizeof(PktRXprogram *) * progRegMaxCount);
	progRegsArrayBacking = malloc(sizeof(PktRXprogram) * progRegMaxCount);

	for (i=0; i < progRegMaxCount; i++) {
		progRegs[i] = (PktRXprogram *)((uint8_t *)progRegsArrayBacking + sizeof(PktRXprogram)*i);
		progRegs[i]->progID = 0xFF;
		progRegs[i]->callback = NULL;
	}
}

void Pkt::begin()
{
	setTXqueueDepth(txQueueDepth);
	setMaxPrograms(progRegMaxCount);

	if (!do_deepsleep_after_tx)
		xcvr->enableRX();  // Start RX
}

void Pkt::end()
{
	if (txQueue != NULL)
		free(txQueue);
	if (progRegs != NULL)
		free(progRegs);
	xcvr->end();
}

/* Packet TX logic */
void Pkt::setModeTXonly(boolean yesno)
{
	do_deepsleep_after_tx = yesno;
	if (do_deepsleep_after_tx) {
		xcvr->disableRX();
		xcvr->deepsleep();
	} else {
		if (xcvr->radioState() != ENRF24_STATE_PRX) {
			xcvr->enableRX();
		}
	}
}

boolean Pkt::send(const uint8_t progID, const uint8_t *rfaddr, const int len, const void *buffer)
{
	unsigned int i;

	if (progID == 0xFF || rfaddr == NULL || (len && (buffer == NULL)))
		return false;

	for (i=0; i < txQueueDepth; i++) {
		if (txQueue[i]->progID == 0xFF) {
			break;
		}
	}
	if (i == txQueueDepth)
		return false;  // No available queue slots; must run .flush() to send what we have so far.
	
	txQueue[i]->progID = progID;
	memcpy(txQueue[i]->rfaddr, rfaddr, 5);
	txQueue[i]->pktlen = len;
	if (len)
		memcpy(txQueue[i]->buffer, (const uint8_t *)buffer, len);

	return true;
}

void Pkt::flush(void)
{
	unsigned int i, fidx;
	uint8_t *cur_rfaddr, rfbuf[32], sz8;
	int plen=0;

	// Find out if TX queue has any entries; if not, clear, if so, preload cur_rfaddr pointer position
	for (i=0; i < txQueueDepth; i++) {
		if (txQueue[i]->progID != 0xFF) {
			cur_rfaddr = txQueue[i]->rfaddr;
			break;
		}
	}
	if (i == txQueueDepth)
		return;  // Nothing to transmit!
	fidx = i;
	xcvr->purge();  // A precaution.

	do {
		if (!memcmp(txQueue[fidx]->rfaddr, cur_rfaddr, 5)) {
			if (txQueue[i]->pktlen > PKT_MAX_PKTLEN) {  // Cull invalid-sized packets from queue
				txQueue[fidx]->progID = 0xFF;
			} else {
				if ( (txQueue[fidx]->pktlen + 2 + plen) <= 32 ) {
					rfbuf[plen++] = txQueue[fidx]->progID;
					sz8 = txQueue[fidx]->pktlen & 0xFF;
					rfbuf[plen++] = sz8;
					if (sz8) {
						memcpy(&rfbuf[plen], txQueue[fidx]->buffer, sz8);
						plen += sz8;
					}
					// Delete queue entry; it's been handled
					txQueue[fidx]->progID = 0xFF;
				} else {
					// Buffer full; transmit at once!
					Serial.print("[1] Sending RF packet of "); Serial.print(plen); Serial.println(" bytes.");
					xcvr->setTXaddress(cur_rfaddr);
					xcvr->write(rfbuf, plen);
					xcvr->flush();
					plen = 0;
				} /* if (length of current packet plus previous will fit in one 32-byte nRF24 frame) */
			} /* if (current packet is a valid length) */
			fidx++;
		} else {
			// New address; flush packet if necessary
			if (plen) {
				Serial.print("[2] Sending RF packet of "); Serial.print(plen); Serial.println(" bytes.");
				xcvr->setTXaddress(cur_rfaddr);
				xcvr->write(rfbuf, plen);
				xcvr->flush();
				plen = 0;
			}
			// Set cur_rfaddr to this new address, and do not increment fidx so we loop back to process it
			// using the existing code above.
			cur_rfaddr = txQueue[i]->rfaddr;
		} /* if (current packet matches our current RF address) */
	} while (fidx < txQueueDepth && txQueue[fidx]->progID != 0xFF);

	if (plen) {  // Unfinished buffer waiting to be sent
		Serial.print("[3] Sending RF packet of "); Serial.print(plen); Serial.println(" bytes.");
		xcvr->setTXaddress(cur_rfaddr);
		xcvr->write(rfbuf, plen);
		xcvr->flush();
	}
	if (do_deepsleep_after_tx)
		xcvr->deepsleep();
}

/* Packet RX logic */
boolean Pkt::available(void)
{
	return xcvr->available(false);
}

void Pkt::loop(void)
{
	unsigned int i, j, plen;
	boolean found_reg;
	uint8_t rfbuf[32];

	while (xcvr->available(false)) {
		plen = xcvr->read(rfbuf, 32);
		for (i=0; i < plen; i++) {
			if (rfbuf[i] && rfbuf[i] != 0xFF) {
				// Only process this packet if the length does not send us past the buffer!
				// (otherwise this would make an easy buffer overflow attack vector)
				if ((i + rfbuf[i+1] + 1) < plen) {
					// Search program registrations for this program ID, or run the UnknownProgram callback if present.
					found_reg = false;
					for (j=0; j < progRegMaxCount; j++) {
						if (progRegs[j]->progID == rfbuf[i]) {
							Serial.print("Packet of "); Serial.print(rfbuf[i+1], DEC); Serial.print(" bytes found with registered progID="); Serial.print(rfbuf[i], DEC); Serial.print(", executing callback at 0x"); Serial.println((uint32_t)(progRegs[j]->callback), HEX);
							found_reg = true;
							progRegs[j]->callback((const uint8_t)rfbuf[i], (const int)rfbuf[i+1], (const void *)&rfbuf[i+2]);
						}
					}
					if (!found_reg) {
						Serial.print("Packet of "); Serial.print(rfbuf[i+1], DEC); Serial.print(" bytes found with progID="); Serial.print(rfbuf[i], DEC); Serial.println(" but no registered callback found.");
					}
					if (!found_reg && unknownProgramCallback != NULL) {
						Serial.println("Executing unknownProgramCallback-");
						unknownProgramCallback((const uint8_t)rfbuf[i], (const int)rfbuf[i+1], (const void *)&rfbuf[i+2]);
					}
					// Run the defaultCallback (attachAllPrograms()) if defined
					if (defaultCallback != NULL) {
						defaultCallback((const uint8_t)rfbuf[i], (const int)rfbuf[i+1], (const void *)&rfbuf[i+2]);
					}
					// Flush this packet and advance
					i += rfbuf[i+1] + 1;  // note; for() logic will advance 'i' one more
				} else {
					// Invalid packet length field; we're done.
					Serial.println("RF packet found with invalid length field; dropping");
					i = plen;
				} /* current packet's length is valid */
			} /* rfbuf[i] is a valid progID field */
		} /* for(i = 0 to plen) */
	} /* while (nRF24 RX FIFO isn't empty) */
}

/* Program ID callback registration */
boolean Pkt::attachProgram(const uint8_t progID, PKT_CALLBACK callback)
{
	unsigned int i;

	if (!progID || progID == 0xFF)
		return false;  // Invalid progID

	for (i=0; i < progRegMaxCount; i++) {
		if (progRegs[i]->progID == progID)
			return false;  // Callback already registered for this program ID.
	}

	for (i=0; i < progRegMaxCount; i++) {
		if (progRegs[i]->progID == 0xFF) {
			progRegs[i]->progID = progID;
			progRegs[i]->callback = callback;
			return true;
		}
	}
	return false;  // Ran out of slots for program ID registrations
}

boolean Pkt::detachProgram(const uint8_t progID)
{
	unsigned int i;

	if (!progID || progID == 0xFF)
		return false;  // Invalid progID
	
	for (i=0; i < progRegMaxCount; i++) {
		if (progRegs[i]->progID == progID) {
			progRegs[i]->progID = 0xFF;
			progRegs[i]->callback = NULL;
			return true;
		}
	}
	return false;  // progID not found in registration array
}

boolean Pkt::attachUnknownProgram(PKT_CALLBACK callback)
{
	if (callback == NULL || unknownProgramCallback != NULL)
		return false;
	
	unknownProgramCallback = callback;
	return true;
}

boolean Pkt::detachUnknownProgram(void)
{
	if (unknownProgramCallback == NULL)
		return false;
	
	unknownProgramCallback = NULL;
	return true;
}

boolean Pkt::attachAllPrograms(PKT_CALLBACK callback)
{
	if (callback == NULL || defaultCallback != NULL)
		return false;

	defaultCallback = callback;
	return true;
}

boolean Pkt::detachAllPrograms(void)
{
	if (defaultCallback == NULL)
		return false;

	defaultCallback = NULL;
	return true;
}
