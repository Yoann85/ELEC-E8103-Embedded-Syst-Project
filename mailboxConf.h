/*
 * mailboxConf.h
 *
 *  Created on: 7 Dec 2019
 *      Author: Roope
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <ti/drivers/Board.h>

#ifndef MAILBOXCONF_H_
#define MAILBOXCONF_H_

#define NUMMSGS         5

typedef struct MsgObj {
    Int     id;
    Char    message[30];
} MsgObj;

typedef struct MailboxMsgObj {
    Mailbox_MbxElem  elem;
    MsgObj           obj;
} MailboxMsgObj;

MailboxMsgObj mailboxBuffer[NUMMSGS];

Mailbox_Struct mbxStruct;
Mailbox_Handle mbxHandle;

#endif /* MAILBOXCONF_H_ */
