#include <stdint.h>
#ifndef ETHER_SERVER_LOOP_H
#define ETHER_SERVER_LOOP_H

extern int CommInit(struct lg_master *pLgMaster);
extern int CommConfigSockfd(struct lg_master *pLgMaster);
extern int DoProcEnetPackets(struct lg_master *pLgMaster);
extern int SendConfirmation(struct lg_master *pLgMaster, unsigned char theCommand);
extern void SendA3(struct lg_master *pLgMaster);
extern void HandleResponse (struct lg_master *pLgMaster, int32_t lengthOfResponse, uint32_t gRespondToWhom );
extern int CheckInput(struct lg_master *pLgMaster);

#define kRespondExtern          0U
#define kDoNotRespond           5U
#define TEST_TARGET_FIND 1
#define COMM_CONFIRM_LEN 6
#define COMM_A3_LEN      1
#define COMM_RESP_ERROR_LEN1  (sizeof(uint32_t) + 2)
#define COMM_RESP_MIN_LEN  (sizeof(uint32_t) + 2)
#define COMM_RESP_MAX_LEN     4096
#define COMM_RECV_MIN_LEN  6
#define COMM_ETHER       2
#define COMM_SERIAL      1
#define COMM_MAX_ENET_RETRIES  4
#define COMM_RECV_MAX_SIZE    0x4000   // Max payload is 8192, but need space for header & CRC.  Double to be on safe side to 16k
#endif
