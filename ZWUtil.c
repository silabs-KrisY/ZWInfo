/* ************ZWUtil.c***************************************************
 * ZWUtil provides a very minimal example utility to interface to the Z-Wave
 * SerialAPI to read serialAPI information from the device.
 * USAGE: ZWUtil <uart port>
 */

#include <stdio.h>
#include <fcntl.h>      /* File Control Definition */
#include <termios.h>    /* Terminal Control Definition */
#include <unistd.h>     /* read/write definition */
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include "ZWave.h"

// RX/TX UART buffer size. Most Z-Wave frames are under 64 bytes.
#define BUF_SIZE 128

void usage() {
    printf("\nZWUtil <uart port>\n");
    printf("ZWUtil will query the Z-Wave SerialAPI node on the specified uart port\n");
}

int serialapi_device;
char readBuf[BUF_SIZE];
char sendBuf[BUF_SIZE];

unsigned char checksum(char *pkt, int len) { /* returns the checksum of PKT */
    int i;
    int sum=0xff;
    for(i=0;i<len;i++) {
        sum ^= *(pkt+i);
    }
    return(sum&0x0FF);
}

int GetSerial(char *pkt) { /* Get SerialAPI frame from UART. Returns the length of pkt and data in PKT */
    /* strips the SOF/LEN/Type and checksum and ACKs the frame if the checksum is OK */
    int i,j,len,type,index;
    char ack=ACK;
    int searching=true;
    i=0;

    while (searching) {                     // Seach for the SOF is complete when we find the SOF and type fields
        for (j=0; i<1 && j<250; j++) {      // wait up to about 250ms - if waiting for a routed frame this might need to be longer
            i=read(serialapi_device,pkt,1);
            if (i==1 && readBuf[0]!=SOF) i=0;  // drop anything until SOF
            usleep(1000);                   // Wait 1ms
        }
        if (i!=1) {
            return(-1);   // no frame so just return -1
        }
        i=read(serialapi_device,pkt,1);                  // LEN (includes LEN and TYPE)
        len=pkt[0];
        i=read(serialapi_device,pkt,1);                  // TYPE (must be 00 or 01)
        type=pkt[0];
        if (type<=0x01) searching=false;    // TODO could also qualify that LEN is less than 64?
    }
    i=0; index=0;
    for (j=0; j<1000 && index<(len-1); j++) {
        i=read(serialapi_device,&pkt[index],len);
        if (i<1) {                          // Nothing yet
            usleep(100);                    // Wait 100uS
        }
        if (i>0) {
            index+=i;
        }
    }
    // TODO add the checksum check here
    write(serialapi_device,&ack,1);   // ACK the frame
    return(len-2);      // remove LEN and TYPE
} /* GetSerial */

int SendSerial(const char *pkt,int len) { /* send SerialAPI command PKT of length LEN after encapsulating */
    /* Returns ACK/NAK/CAN if the packet was delivered to the serialapi_device, -1 if no response */
    /* if not ACKed, will try a total of 3 times */
    /* SerialAPI format:
     * SOF      =0x01
     * LEN      =Number of bytes in this frame not including SOF and CHECKSUM (or all bytes-2)
     * Type     =0x00=Request, 0x01=Response
     * FUNCID   =SerialAPI command byte
     * PARAMS(n)=Command parameters and/or data bytes
     * CHECKSUM = XOR of all bytes except SOF. Should be 0 if checksum is OK. NAK is sent if checksum fails.
     */
    char buf[BUF_SIZE];
    int i,j;
    int retry;
    int ack=ACK;
    buf[0]=SOF;
    buf[1]=len+2; // add LEN, TYPE
    buf[2]=REQUEST;
    memcpy(&buf[3],pkt,len);
    buf[len+3]=checksum(&buf[1],len+2);
#ifdef DEBUG // tx debug
    printf("Sending");
    for (i=0;i<(len+4); i++) {
        printf(" %02X",buf[i]);
    }
    printf("\n");
#endif
    for (retry=1;retry<=3;retry++) {    // retry up to 3 times
        tcflush(serialapi_device,TCIFLUSH);  // purge the UART Rx Buffer
        write(serialapi_device,buf,len+4);   // Send the frame to the serialapi_device
        i=0;
        for (j=0; i<1 && j<10000; j++) {
            i=read(serialapi_device,readBuf,1);          // Get the ACK/NAK/CAN
        }
        if (i==1) {
            if (readBuf[0]==ACK) break; // Got the ACK so return
            write(serialapi_device,&ack,1);          // Got something else so try sending an ACK to clear
        }
        sleep(2);      // wait a bit and try again
    }
    if (i!=1) {
        printf("UART Timeout\r\n");
        return(-1);
    }
    return(readBuf[0]);
} /* SendSerial */

const char* statusString(int status) {
  if (status == -1) {
    return "NO_RESPONSE";
  } else if (status == ACK) {
    return "NAK";
  } else if (status == CAN) {
    return "CAN";
  } else {
    return "UNKNOWN";
  }
}

int main(int argc, char *argv[]) { /*****************MAIN*********************/
    struct termios Settings;

    int ack,len,i,j;

    if (argc!=2) {
        usage();
        return(-1);
    }

    /* setup the UART to 115200, 8 bits, no parity, 2 stop bit. */
    serialapi_device = open(argv[1],O_RDWR | O_NOCTTY | O_NONBLOCK); /* serialapi_device plugged into a Linux machine is normally at /dev/ttyACM0 */
    if (serialapi_device<0) {
      printf("Error opening %s\r\n",argv[1]);
      return(-1);
    }
    tcgetattr(serialapi_device,&Settings);
    cfsetispeed(&Settings,B115200);
    cfsetospeed(&Settings,B115200);
    Settings.c_cflag &= ~PARENB;
    Settings.c_cflag &= ~CSTOPB;
    Settings.c_cflag &= ~CSIZE;
    Settings.c_cflag |= CS8;
    Settings.c_cflag &= ~CRTSCTS;
    Settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    Settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    Settings.c_oflag &= ~OPOST;
    Settings.c_cc[VMIN] = BUF_SIZE;
    Settings.c_cc[VTIME] = 1; // wait this many 100mSec if no chars are available yet - need to wait for the ACK
    if (tcsetattr(serialapi_device,TCSANOW,&Settings)!=0) {
        printf("Unable to set serialapi_device attributes\r\n");
        return(-2);
    }

    printf("Querying SerialAPI device on %s...\r\n", argv[1]);
    sendBuf[0]=FUNC_ID_SERIAL_API_GET_INIT_DATA;
    ack=SendSerial(sendBuf,1);
    if (ack!=ACK) {
        printf("Unable to send Z-Wave SerialAPI command API_GET_INIT_DATA" \
        " (%s)\r\n",statusString(ack));
    } else {
        len=GetSerial(readBuf);
        if (len<20 || readBuf[0]!=FUNC_ID_SERIAL_API_GET_INIT_DATA) {
            printf("Incorrect response = %02X, %02X\r\n",len,readBuf[0]);
            printf("%02X, %02X\r\n",readBuf[1],readBuf[2]);
        } else {
            printf("SerialAPI protocol version=%d\r\n",readBuf[1]);
            if (readBuf[2]&0x04) printf("Secondary ");
            else                 printf("Primary ");
            if (readBuf[2]&0x01) printf("Slave ");
            else                 printf("Controller ");
            if (readBuf[2]&0x08) printf("SIS ");
            else                 printf("    ");
            printf("\nNodes:");
            for (i=0;i<readBuf[3];i++) {
                for (j=0;j<8;j++) {
                    if (readBuf[i+4]&(1<<j)) {
                        printf(" %03d", i*8+j+1);
                    }
                }
            }
            printf("\r\n");
            i = readBuf[3] + 4;
            printf("chip_type=0x%x, chip_version=0x%x\r\n",
                readBuf[i], readBuf[i+1]);
        }
    }
    // Now let's get capabilities
    sendBuf[0]=FUNC_ID_SERIAL_API_GET_CAPABILITIES;
    ack=SendSerial(sendBuf,1);
    if (ack!=ACK) {
        printf("Unable to send Z-Wave SerialAPI command SERIAL_API_GET_CAPABILITIES"
                      " (%s)\r\n",statusString(ack));
    } else {
        len=GetSerial(readBuf);
        if (len<20 || readBuf[0]!=FUNC_ID_SERIAL_API_GET_CAPABILITIES ) {
            printf("Incorrect response = %02X, %02X\n",len,readBuf[0]);
            printf("%02X, %02X\n",readBuf[1],readBuf[2]);
        } else {
            printf("SerialAPI Ver=%d.%d\r\n",readBuf[1], readBuf[2]);
            printf("Product Type/Product ID: 0x%x/0x%x\r\n",
            readBuf[5] << 8 | readBuf[6],
            readBuf[7] << 8 | readBuf[8]);
        }
    }

    close(serialapi_device);
}   /* Main */
