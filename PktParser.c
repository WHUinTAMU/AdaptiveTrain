/** Parse IMU packets containing raw data and RSSI and packet number data.
    Author: Amy
 */
#include "PktParser.h"
long PKT_NUMBER = 1;

unsigned int getLocalTime()
{
    //watchout for 23:59:59.999 -> 0:0:0.000
    SYSTEMTIME now;
    GetSystemTime(&now);
    return(((now.wHour * 60 + now.wMinute) * 60 + now.wSecond) * 1000 + now.wMilliseconds);
}

int toMs(unsigned char a, unsigned char b, unsigned int overflowTime)
{
    float myFlt = ((float)((overflowTime + (a << 8) + b) * 1000)) / 500000.0f;
    unsigned int myInt = (unsigned int)(myFlt + 0.5f);
    return myInt;
}

int uCharToInt(unsigned char a, unsigned char b)
{
    short int myInt = ((a << 8) + b);
    if (myInt > 0x32768)
    {
        myInt = (~myInt)*(-1);
    }
    return myInt;
}

//Read one packet from SerialPort
PktData blockingReadOnePacket(HANDLE hComm)
{
    PktData pktData = { 0.0 };
    // See buffer length.
    COMSTAT ComStat;
    DWORD dwError = 0;
    ClearCommError(hComm, &dwError, &ComStat);
    //printf("buffer len: %d\n", ComStat.cbInQue);

    unsigned char packet[256];
    unsigned char cData = 0;
    unsigned char pData = 0;
    int dataCount = 1;

    // find the start of packet
    while (((pData != DLE) || (cData != SOH)))
    {
        pData = cData;
        cData = readSByte(hComm);
        if ((pData == DLE) && (cData == DLE))
        {
            pData = cData;
            cData = readSByte(hComm);
        }
    }

    // start collect data
    while (((pData != DLE) || (cData != EOT)))
    {
        pData = cData;
        cData = readSByte(hComm);

        if (cData != DLE)
            packet[(dataCount++) % 256] = cData;
        else
        {
            pData = cData;
            cData = readSByte(hComm);
            if (cData == DLE)
            {
                packet[(dataCount++) % 256] = cData;
            }
        }

        if(dataCount > 25)
            return pktData;
    }

    if (dataCount == 25)
    {
        int accX = uCharToInt(packet[1], packet[2]) ;
        int accY = uCharToInt(packet[3], packet[4]);
        int accZ = uCharToInt(packet[5], packet[6]);
        int gyroX = uCharToInt(packet[7], packet[8]);
        int gyroY = uCharToInt(packet[9], packet[10]);
        int gyroZ = uCharToInt(packet[11], packet[12]);
        int magX = uCharToInt(packet[13], packet[14]);
        int magY = uCharToInt(packet[15], packet[16]);
        int magZ = uCharToInt(packet[17], packet[18]);

        pktData.rssiData1 = packet[19];
        pktData.rssiData2 = packet[20];
        pktData.rssiData3 = packet[21];
        pktData.rssiData4 = packet[22];

        pktData.accX = accX / 4096.0 / 4 / 2 ;
        pktData.accY = accY / 4096.0 / 4 / 2 ;
        pktData.accZ = accZ / 4096.0 / 4 / 2 ;
        pktData.gyroX = gyroX / 65.5 / 120;
        pktData.gyroY = gyroY / 65.5 / 120;
        pktData.gyroZ = gyroZ / 65.5 / 120;
        pktData.magX = magX * 1.0;
        pktData.magY = magY * 1.0;
        pktData.magZ = magZ * 1.0;

        pktData.pktNumber = (PKT_NUMBER ++);
        pktData.timeStamp = getLocalTime();

//      if(packet[19] != 0 || packet[20] != 0 || packet[21] != 0 || packet[22] != 0)
//            printf("%ld, %d, %u, %u, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d, %d, %d, %d \n",
//                   pktData.timeStamp, pktData.pktNumber, packet[24], packet[23],
//                   pktData.accX, pktData.accY, pktData.accZ,
//                   pktData.gyroX, pktData.gyroY, pktData.gyroZ,
//                   pktData.magX, pktData.magY, pktData.magZ,
//                   pktData.rssiData1, pktData.rssiData2, pktData.rssiData3, pktData.rssiData4);
    }
    return pktData;
}
