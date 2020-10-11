//
//  RRCI.h
//  CRRCI
//
//  Created by Rodolphe Pineau on 2020-10-9
//  RRCI X2 plugin

#ifndef __RRCI__
#define __RRCI__
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>

#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

#ifdef SB_WIN_BUILD
#include <time.h>
#endif

#include <math.h>
#include <string.h>

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"

#define DRIVER_VERSION      1.0

// #define PLUGIN_DEBUG 3

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 5000

// error codes
enum RRCIErrors {PluginOK=0, NOT_CONNECTED, CANT_CONNECT, BAD_CMD_RESPONSE, COMMAND_FAILED, NO_DATA_TIMEOUT};

// Error code
enum RRCIRoofState {OPEN=0, MOVING, CLOSED , UNKNOWN};
enum RoofAction {IDLE=0, OPENING, CLOSING};


class CRRCI
{
public:
    CRRCI();
    ~CRRCI();

    int         Connect(const char *szPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; }

    void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void        setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper; };

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double dNewAz);
    int openRoof();
    int closeRoof();
	int	findHome();

    // command complete functions
    int isGoToComplete(bool &bComplete);
    int isOpenComplete(bool &bComplete);
    int isCloseComplete(bool &bComplete);
    int isParkComplete(bool &bComplete);
    int isUnparkComplete(bool &bComplete);
    int isFindHomeComplete(bool &bComplete);

    double getCurrentAz();
    double getCurrentEl();

    int abortMove();
    
protected:

    SleeperInterface    *m_pSleeper;
    bool            m_bIsConnected;
    SerXInterface   *m_pSerx;
    int             m_nRoofState;
    double          m_dCurrentAzPosition;
    double          m_dCurrentElPosition;
    bool            m_bSafe;
    int             m_RoofAction;
    int             readResponse(char *szRespBuffer, int nBufferLen, int nTimeout = MAX_TIMEOUT);
    int             domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen, int nTimeout = MAX_TIMEOUT);
    int             getState();
    int             parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator);
    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);
    std::string     findField(std::vector<std::string> &svFields, const std::string& token);

    
#ifdef PLUGIN_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;      // LogFile
#endif

};

#endif
