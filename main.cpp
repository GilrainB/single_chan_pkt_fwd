/*******************************************************************************
 *
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 *******************************************************************************/

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <sys/stat.h>
#include <errno.h>

#include <sys/ioctl.h>
#include <net/if.h>

using namespace std;

#include "base64.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>

typedef bool boolean;
typedef unsigned char byte;

extern int errno;
static const int CHANNEL = 0;
FILE *csvFile = NULL;

byte currentMode = 0x81;

char message[256];
char b64[256];
int b64_cnt = 0;

bool sx1272 = true;

uint8_t receivedbytes;

struct sockaddr_in si_other;
int s, slen=sizeof(si_other);
struct ifreq ifr;

// Collect metadata on received packets; Amount of received packets...
// Total
// With OK CRC
// With bad CRC
// Without CRC
// Amount of packets forwarded(?)
uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

//////////////////////////////////
/////// LoRaWAN data types ///////
//////////////////////////////////
// Receive properties
enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };
// Normal BW is BW125
enum LoRaChan {LoRaChan_0=868100000, LoRaChan_1=868300000, LoRaChan_2=868500000, LoRaChan_Test_0=869462500, LoRaChan_Test_1=869587500, LoRaChan_Test_0_BW250=869525000 };

// Bitfields are assigned from least to most significant; reversed from the 'real' situation, according to sources
// DevAddr has been verified readeable(which means FHDR_T is correct)
// The reversed struct seemed to be the correct working version;
// When a node was sending packets with ADR on, The reversed struct(second) would output FOptsLen 0, but the non-reversed struct returned 8. 
// Therefore will the second struct be used as FCtrl_t
/*
typedef struct {                // Frame Control, uplink structure
        bool ADR:1;             // Adaptive Data Rate control
        bool ADRACKReq:1;       // Request for acknowledgment (on this message)
        bool ACK:1;                     // When receiving a confirmed data message, the receiver shall respond with a data frame that has this acknowledgment bit (ACK) set.
        bool FPending:1;        // Frame pending bit, only used in downlink communication(so ignore for uplink info from Nodes).
        // Indicating that the gateway has more data pending to be sent; it's therefore asking the end-device to open another receive window ASAP by sending another uplink message.
        uint8_t FOptsLen:4;     // Length of the frame options. Can be 0 to 15 bytes. This amount is the offset for the actual data following FCnt in FHDR_t.
} FCtrl_t;
*/

typedef struct { 		// Frame Control, uplink structure reversed
	uint8_t FOptsLen:4;	// Length of the frame options. Can be 0 to 15 bytes. This amount is the offset for the actual data following  FCnt in FHDR_t.
	bool FPending:1;	// Frame pending bit, only used in downlink communication(so ignore for uplink info from Nodes).
	// Indicating that the gateway has more data pending to be sent it's therefore asking the end-device to open another receive window ASAP by sending another uplink message.
	bool ACK:1;			// When receiving a confirmed data message, the receiver shall respond with a data frame that has this acknowledgment bit (ACK) set.
	bool ADRACKReq:1; 	// Request for acknowledgment (on this message)
	bool ADR:1; 		// Adaptive Data Rate control
} FCtrl_t;

typedef struct {
        uint32_t DevAddr;       // Device address --> Verified
        FCtrl_t FCtrl;          // Frame Control
        uint16_t FCnt;          // Frame counter
	// Not included: FOpts; these could not have been included in the frame
} FHDR_t;

// Dissection of a real Payload: 0x'80451901268033000F324B702308C69D49BA0B6FF5A77E285480878377'

// 80 MHDR(MType, RFU en Major) = Confirmed Data Up, -, 0

// FHDR_t
// 45190126 --> 0x26011945 DevAddr
// 80 FCtrl_t --> ACK request(confirmed msg)
// 3300 --> 0x0033 FCnt = 57(dec)

// Port 0x0F = 15
// Data 324B702308C69D49BA0B6FF5A77E2854
// MIC 80878377

/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF7;

// Set center frequency
uint32_t  freq = 868100000; // in Mhz! (868.1)

#define CSV_D "\t" // CSV delimiter for in between data/columns
#define PRINT_DATA_ON_CLI 1
#define PRINT_RAW_DATA 1

// define servers
#define GATEWAY_CONNECTED_TO_TTN 0
#if GATEWAY_CONNECTED_TO_TTN
// TODO: use host names and dns
#define SERVER1 "54.72.145.119"    // The Things Network: croft.thethings.girovito.nl
//#define SERVER2 "192.168.1.10"      // local
#define PORT 1700                   // The port on which to send data

// Set location
float lat=0.0;
float lon=0.0;
int   alt=0;

/* Informal status fields */
static char platform[24]    = "Single Channel Gateway";  /* platform definition */
static char email[40]       = "";                        /* used for contact email */
static char description[64] = "";                        /* used for free form description */

#endif

// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

#define BUFLEN 2048  //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE	  1024

// ESSENTIALS

void die(const char *s)
{
    perror(s);
    exit(1);
}

#define selectreceiver() digitalWrite(ssPin, LOW)

#define unselectreceiver() digitalWrite(ssPin, HIGH)

byte readRegister(byte addr)
{
	//unsigned char spibuf[2];
    union {
		unsigned char uch[2];
		uint16_t u16;
	} spibuf;

    selectreceiver();
	
    // spibuf[0] = addr & 0x7F;
    // spibuf[1] = 0x00;
	spibuf.u16 = addr & 0x007F;
	
    wiringPiSPIDataRW(CHANNEL, spibuf.uch, 2);
    unselectreceiver();

    return spibuf.uch[1];
}

void writeRegister(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}

// END ESSENTIALS
// LoRa hardware functions

boolean receivePkt(char *payload)
{

    // clear rxDone
    writeRegister(REG_IRQ_FLAGS, 0x40);

    int irqflags = readRegister(REG_IRQ_FLAGS);

    cp_nb_rx_rcv++;

    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        writeRegister(REG_IRQ_FLAGS, 0x20);
		cp_nb_rx_bad++;
        printf("CRC error\n");
        return false;
    } else {

        cp_nb_rx_ok++;

        byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

#if PRINT_RAW_DATA
	printf("Payload: 0x'");
#endif

        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readRegister(REG_FIFO);

#if PRINT_RAW_DATA
            fprintf(stdout, "%02X", payload[i]);
        }
	printf("'\n");
#else
        }
#endif
    }
    return true;
}

void SetupLoRa(); // Prototype

void SetupLoRa_(uint32_t frequency, sf_t spread){
		freq = frequency;
		sf = spread;
		SetupLoRa();
}

void SetupLoRa()
{
    
    digitalWrite(RST, HIGH);
    delay(1);
    digitalWrite(RST, LOW); 
    delay(2);

    byte version = readRegister(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
		// Datasheet said low 100usec, high 5 ms
		// Currently already low, so skip setting and waiting
        //digitalWrite(RST, LOW);
        //delay(2);
        digitalWrite(RST, HIGH);
        delay(10);
        version = readRegister(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }
	puts("\n");

    writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeRegister(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG,0x0B);
        } else {
            writeRegister(REG_MODEM_CONFIG,0x0A);
        }
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
		/*
		REG_MODEM_CONFIG3
		bit 3 LowDataRateOptimize; 0 = Disabled, 1 = Enabled; mandated for when the symbol length exceeds 16ms
		bit 2 AgcAutoOn; 0 = LNA gain set by register LnaGain, 1 = LNA gain set by the internal AGC loop
		 */
        if (sf == SF11 || sf == SF12) {
            writeRegister(REG_MODEM_CONFIG3,0x0C);
        } else {
            writeRegister(REG_MODEM_CONFIG3,0x04);
        }
        writeRegister(REG_MODEM_CONFIG,0x72);
		//
		// 0x72= 0111 001 0
		// 0111 = BW125 kHz
		// 001 = codingrate 4/5
		// 0 = Explicit mode('automatic', no filtering on packet header)
        writeRegister(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeRegister(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    writeRegister(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeRegister(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeRegister(REG_HOP_PERIOD,0xFF);
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

    // Set Continous Receive Mode
    writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
    writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);
	
	// initialize FILE
	{
		struct tm *currtime;
		const char * s_fileDirectory = "/home/pi/LoRaWAN-TESTGateway";
		char s_buf[20], // 4+2+2+1+2+2+2(YYYYmmdd-HHMMSS) = 15 + '\0' = 16
			filename[sizeof(s_fileDirectory) + sizeof(s_buf) + 5];
		
		if(csvFile != NULL) fclose(csvFile);
		
		time_t t = time(NULL);
		currtime = localtime(&t);
		if (currtime == NULL) {
			perror("couldn't get localtime");
			exit(EXIT_FAILURE);
		}
		strftime(s_buf, sizeof(s_buf), "%Y%m%d-%H%M%S", currtime);
		// s_buf is filled with the current date and time(Japanese format with '-' in between)
		
		sprintf(filename, "%s/%s.csv", s_fileDirectory, s_buf);
		//strcpy(filename, s_fileDirectory);
		//strcat(filename, s_buf);
		//strcat(filename, ".csv");
		//printf("Filename='%s'\ns_buf='%s'\n", filename, s_buf);
		
		if(mkdir(s_fileDirectory, 0700/*RW user, group others none*/ ) == 0 )
			printf("Created directory '%s'\n", s_fileDirectory); // usually directory exists
		else
		switch (errno){
			case EEXIST:
				printf("Directory '%s' exists.\n", s_fileDirectory);
				break;
			default:
				fprintf(stderr, "Can't access '%s' directory.", s_fileDirectory);
				exit(1);
		}
		
		printf("Opening file '%s'", filename);
		csvFile = fopen(filename, "w");
		printf("...");
		fprintf(csvFile,
		"Packetno."
		CSV_D "TimeEpoch"
		CSV_D "SignalNoiseRatio"
		CSV_D "RSSI"
		CSV_D "RSSI Packet"
		CSV_D "Length"
		CSV_D "Type" // see MType of the MHDR
		CSV_D "IsLoRaWAN-Data"
		CSV_D "FCtrl"
		CSV_D "Address"
		CSV_D "Framecounter"
		CSV_D "Port"
		"\r\n"
		);
		printf("\tSuccess!\n");
	}
	
    printf("Started listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
    printf("------------------\n");
}
// END LoRa hardware functions

#if GATEWAY_CONNECTED_TO_TTN

void sendudp(char *msg, int length) {

//send the update
#ifdef SERVER1
    inet_aton(SERVER1 , &si_other.sin_addr);
    if (sendto(s, (char *)msg, length, 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        die("sendto()");
    }
#endif

#ifdef SERVER2
    inet_aton(SERVER2 , &si_other.sin_addr);
    if (sendto(s, (char *)msg, length , 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        die("sendto()");
    }
#endif
}

void sendstat() {

    static char status_report[STATUS_SIZE]; /* status report as a JSON object */
    char stat_timestamp[24];
    time_t t;

    int stat_index=0;

    /* pre-fill the data buffer with fixed fields */
    status_report[0] = PROTOCOL_VERSION;
    status_report[3] = PKT_PUSH_DATA;

    status_report[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
    status_report[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
    status_report[6] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
    status_report[7] = 0xFF;
    status_report[8] = 0xFF;
    status_report[9] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
    status_report[10] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
    status_report[11] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];

    /* start composing datagram with the header */
    uint8_t token_h = (uint8_t)rand(); /* random token */
    uint8_t token_l = (uint8_t)rand(); /* random token */
    status_report[1] = token_h;
    status_report[2] = token_l;
    stat_index = 12; /* 12-byte header */

    /* get timestamp for statistics */
    t = time(NULL);
    strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));

    int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE-stat_index, "{\"stat\":{\"time\":\"%s\",\"lati\":%.5f,\"long\":%.5f,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%.1f,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}", stat_timestamp, lat, lon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, (float)0, 0, 0,platform,email,description);
    stat_index += j;
    status_report[stat_index] = 0; /* add string terminator, for safety */

    printf("stat update: %s\n", (char *)(status_report+12)); /* DEBUG: display JSON stat */

    //send the update
    sendudp(status_report, stat_index);

}
#endif // END GATEWAY_CONNECTED_TO_TTN

// Convenience functions
// Writes to file and to stdout
void csvWriteLongInt(long int data, const char * description){
	fprintf(csvFile, CSV_D "%li", data); 
	printf(" %s %li", description, data);
} 
void csvWriteHex(uint32_t data, const char * description){
	fprintf(csvFile, CSV_D "\"%X\"", data); 
	printf("; %s 0x%X", description, data);
} 

/// This function collects information on the LoRaWAN payload, if present.
/// Data: Device address, frame count, port(0x01-0xDF)
void readLoRaMacPayload(char* macpayload){
	FHDR_t  * header 		= (FHDR_t * ) macpayload;
	uint8_t   offset 		= header->FCtrl.FOptsLen;
	uint8_t * port   		= ((uint8_t *)(header + 1)) + offset; // if port is present, it follows the FOpts
	uint8_t * EncryptedData	= port + 1; // If port field is present, skip the port field, else the data starts at port, if data is present
	
	// printf(" FCtrl 0x%X", *((uint8_t*) (&(header->FCtrl))) ); // Turn header->FCtrl into an uint8 pointer, then dereference it.
	csvWriteHex(*((uint8_t*) (&(header->FCtrl))), "FCtrl" );
	//printf(" %d Offset(or reversed %d)", (int)offset,  (( (FCtrl_rev_t*) (&(header->FCtrl)) )->FOptsLen)  );
	printf(" Offset[%d]", (int)offset);
	
	// Write CSV
	csvWriteHex(header->DevAddr, "Address"); //fprintf(csvFile, CSV_D "%u", header->DevAddr);
	csvWriteLongInt(header->FCnt, "Framecounter");
	csvWriteHex(*port, "Port");
	// Show EncryptedData
	b64_cnt = bin_to_b64((uint8_t *)EncryptedData, (macpayload + receivedbytes) - (char*)EncryptedData, (char *)(b64), 341); // Last 4 bytes are the MIC
	
}

void readMessage_LoRaWAN(char* payload){
	// Using data from LoRaWAN specification, 4 MAC Message Formats, page 15.
	
	bool hasLoRaWanData = false;
	char const * nextCsvStr = NULL;
	char payload_MHDR = payload[0];
	char * payload_MACPayload = payload + 1;
	uint32_t payload_MIC = *((uint32_t*) (payload + receivedbytes - 4));
	
	/*
	Size (bytes)|	1   	1..M      	4
	PHYPayload  |	MHDR	MACPayload 	MIC
	*/
	/* MHDR: 
	Bit# 	7..5	4..2	1..0
	MHDR	MType	RFU 	Major
	*/
	/* MType = 
		000 Join Request
		001 Join Accept
		010 Unconfirmed Data Up
		011 Unconfirmed Data Down
		100 Confirmed Data Up
		101 Confirmed Data Down
		110 RFU
		111 Proprietary
	*/
	switch(payload_MHDR>>5) {
		case 0b000:
			nextCsvStr = "Join Request";
			break;
		case 0b001:
			nextCsvStr = "Join Accept";
			break;
		case 0b010:
			nextCsvStr = "Unconfirmed Data Up";
			hasLoRaWanData = true;
			break;
		case 0b011:
			nextCsvStr = "Unconfirmed Data Down";
			break;
		case 0b100:
			nextCsvStr = "Confirmed Data Up";
			hasLoRaWanData = true; // This type of message doesn't need to be looked into as it's not relevant for the test
			break;
		case 0b101:
			nextCsvStr = "Confirmed Data Down";
			break;
		case 0b110:
			nextCsvStr = "RFU";
			break;
		case 0b111:
			nextCsvStr = "Proprietary";
			break;
	}
	fprintf(csvFile, CSV_D "\"%s\"", nextCsvStr); // Message Type(MType)
	printf(", %s", nextCsvStr);
	
	// Mayor bitfield: 00 LoRaWAN R1, else RFU
	if(hasLoRaWanData != false && (payload_MHDR & 0b11) == 0b00) {
		// Is actually LoRaWAN R1 Data
		hasLoRaWanData = true;
	} else {
		hasLoRaWanData = false;
	}
	
	if(hasLoRaWanData){
		fprintf(csvFile, CSV_D "TRUE");
		printf(" with LoRaWAN data;");
	} else {
		fprintf(csvFile, CSV_D "FALSE");
		printf(", NO LoRaWAN data.");
	}
	
	// Check the MACPayload!
	if(hasLoRaWanData) {
		readLoRaMacPayload(payload_MACPayload);
	
#if PRINT_DATA_ON_CLI
		printf(" EncryptedData in base64:\t'");
		fwrite(b64, sizeof(char), b64_cnt, stdout);
		printf("' MIC: 0x%X\n", payload_MIC);
#endif
	}
}

void receivepacket() {

    long int SNR;
    int rssicorr, packetRSSI, RSSI;
	struct timeval now;

    if(digitalRead(dio0) == 1)
        if(receivePkt(message)) {
			
			gettimeofday(&now, NULL);
			
            byte value = readRegister(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {
                rssicorr = 157;
            }

            /*printf("Packet RSSI: %d, ",*/ packetRSSI = (readRegister(0x1A)-rssicorr);//);
            /*printf("RSSI: %d, ",*/ RSSI = (readRegister(0x1B)-rssicorr);//);
            //printf("SNR: %li, ",SNR);
            //printf("Length: %i",(int)receivedbytes);
			
			//
			// Store to csv if message was from testdevice
			//
			/*
			fprintf(csvFile, 
		"Packetno."
		",TimeEpoch"
		",SignalNoiseRatio"
		",RSSI"
		",RSSI Packet"
		",Length"
		"\r\n"
		)
			*/
			//csvWriteLongInt(cp_nb_rx_rcv -1, "no.");
			printf("Packet %u " ,  cp_nb_rx_rcv-1 );
			fprintf(csvFile, "%u", cp_nb_rx_rcv-1 );
			csvWriteLongInt(now.tv_sec, "TimeEpoch");
			csvWriteLongInt(SNR, "SNR");
			csvWriteLongInt(RSSI, "RSSI");
			csvWriteLongInt(packetRSSI, "PacketRSSI");
			csvWriteLongInt(receivedbytes, "Length");
			
			
#if GATEWAY_CONNECTED_TO_TTN 
            b64_cnt = bin_to_b64((uint8_t *)message, receivedbytes, (char *)(b64), 341);
            //fwrite(b64, sizeof(char), j, stdout);
			
            char buff_up[TX_BUFF_SIZE]; /* buffer to compose the upstream packet */
            int buff_index=0;

            /* gateway <-> MAC protocol variables */
            //static uint32_t net_mac_h; /* Most Significant Nibble, network order */
            //static uint32_t net_mac_l; /* Least Significant Nibble, network order */

            /* pre-fill the data buffer with fixed fields */
            buff_up[0] = PROTOCOL_VERSION;
            buff_up[3] = PKT_PUSH_DATA;

            /* process some of the configuration variables */
            //net_mac_h = htonl((uint32_t)(0xFFFFFFFF & (lgwm>>32)));
            //net_mac_l = htonl((uint32_t)(0xFFFFFFFF &  lgwm  ));
            //*(uint32_t *)(buff_up + 4) = net_mac_h;
            //*(uint32_t *)(buff_up + 8) = net_mac_l;

            buff_up[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
            buff_up[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
            buff_up[6] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
            buff_up[7] = 0xFF;
            buff_up[8] = 0xFF;
            buff_up[9] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
            buff_up[10] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
            buff_up[11] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];

            /* start composing datagram with the header */
            uint8_t token_h = (uint8_t)rand(); /* random token */
            uint8_t token_l = (uint8_t)rand(); /* random token */
            buff_up[1] = token_h;
            buff_up[2] = token_l;
            buff_index = 12; /* 12-byte header */

            // TODO: tmst can jump is time is (re)set, not good.
            uint32_t tmst = (uint32_t)(now.tv_sec*1000000 + now.tv_usec);

            /* start of JSON structure */
            memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
            buff_index += 9;
            buff_up[buff_index] = '{';
            ++buff_index;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"tmst\":%u", tmst);
            buff_index += j;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf", 0, 0, (double)freq/1000000);
            buff_index += j;
            memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
            buff_index += 9;
            memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
            buff_index += 14;
            /* Lora datarate & bandwidth, 16-19 useful chars */
            switch (sf) {
            case SF7:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
                buff_index += 12;
                break;
            case SF8:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
                buff_index += 12;
                break;
            case SF9:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
                buff_index += 12;
                break;
            case SF10:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
                buff_index += 13;
                break;
            case SF11:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
                buff_index += 13;
                break;
            case SF12:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
                buff_index += 13;
                break;
            default:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
                buff_index += 12;
            }
            memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
            buff_index += 6;
            memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
            buff_index += 13;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%li", SNR);
            buff_index += j;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%d,\"size\":%u", readRegister(0x1A)-rssicorr, receivedbytes);
            buff_index += j;
            memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
            buff_index += 9;
            j = bin_to_b64((uint8_t *)message, receivedbytes, (char *)(buff_up + buff_index), 341);
            buff_index += j;
            buff_up[buff_index] = '"';
            ++buff_index;

            /* End of packet serialization */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = ']';
            ++buff_index;
            /* end of JSON datagram payload */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = 0; /* add string terminator, for safety */

            printf("rxpk update: %s\n", (char *)(buff_up + 12)); /* DEBUG: display JSON payload */
			

            //send the messages
            sendudp(buff_up, buff_index);
			
#endif // END GATEWAY_CONNECTED_TO_TTN

			readMessage_LoRaWAN(message);
			
			fprintf(csvFile, "\r\n");
			printf("\n");

			fflush(csvFile);
			fflush(stdout);

        } // received a message
		
}

// END Convenience functions

int main () {

    struct timeval nowtime;
#if GATEWAY_CONNECTED_TO_TTN
    uint32_t lasttime;
#endif
	
	while (1) {

		if (nowtime.tv_sec > 1508515200) break; 
		
		if(nowtime.tv_sec != 0) { // If not the first run
			printf("Waiting for System time synchronization.\n");
			delay(3000);
		}
		
		gettimeofday(&nowtime, NULL);
		
	}  // Bigger than 20-10-2017 18:00:00; Human time (GMT): Friday 20 October 2017 16:00:00
	printf("Setting up LoRa TESTGateway.\n");

    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    //int fd = 
    wiringPiSPISetup(CHANNEL, 500000);
    //cout << "Init result: " << fd << endl;

    SetupLoRa_(LoRaChan_0, SF12);

#if GATEWAY_CONNECTED_TO_TTN

    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
        die("socket");
    }
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);

    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);  // can we rely on eth0?
    ioctl(s, SIOCGIFHWADDR, &ifr);

    /* display result */
    printf("Gateway ID: %.2x:%.2x:%.2x:ff:ff:%.2x:%.2x:%.2x\n",
           (unsigned char)ifr.ifr_hwaddr.sa_data[0],
           (unsigned char)ifr.ifr_hwaddr.sa_data[1],
           (unsigned char)ifr.ifr_hwaddr.sa_data[2],
           (unsigned char)ifr.ifr_hwaddr.sa_data[3],
           (unsigned char)ifr.ifr_hwaddr.sa_data[4],
           (unsigned char)ifr.ifr_hwaddr.sa_data[5]);
		   
#endif

    while(1) {

        receivepacket();

#if GATEWAY_CONNECTED_TO_TTN
        gettimeofday(&nowtime, NULL);
        uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
        if (nowseconds - lasttime >= 30) {
            lasttime = nowseconds;
            sendstat();
            cp_nb_rx_rcv = 0;
            cp_nb_rx_ok = 0;
            cp_up_pkt_fwd = 0;
        }
#endif
        delay(1);
    }

    return (0);

}

