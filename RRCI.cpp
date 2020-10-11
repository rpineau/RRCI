//
//  RRCI.cpp
//  CRRCI
//
//  Created by Rodolphe Pineau on 2020-10-9
//  RRCI X2 plugin

#include "RRCI.h"

CRRCI::CRRCI()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

    m_nRoofState = UNKNOWN;
    m_bSafe = false;
    m_RoofAction = IDLE;
    
#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\X2_RRCI.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/X2_RRCI.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/X2_RRCI.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CRRCI::CRRCI] Version %3.2f build 2020_10_10_17400.\n", timestamp, DRIVER_VERSION);
    fprintf(Logfile, "[%s] [CRRCI::CRRCI] Constructor Called.\n", timestamp);
    fflush(Logfile);
#endif

}

CRRCI::~CRRCI()
{

}

int CRRCI::Connect(const char *pszPort)
{
    int nErr = PluginOK;
    char szResp[SERIAL_BUFFER_SIZE];

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::Connect] Connecting to %s.\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 9600 8N1
    if(m_pSerx->open(pszPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    // Upon connection Arduino sends 'RRCI#'
    readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(!strstr(szResp, "RRCI")) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
       fprintf(Logfile, "[%s] [CRRCI::Connect] Error getting RRCI from controller, szResp = %s.\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return ERR_NORESPONSE;
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::Connect] Connected to %s.\n", timestamp, pszPort);
    fprintf(Logfile, "[%s] [CRRCI::Connect] m_bIsConnected = %s.\n", timestamp, m_bIsConnected?"True":"False");
    fprintf(Logfile, "[%s] [CRRCI::Connect] szResp = %s.\n", timestamp, szResp);
    fflush(Logfile);
#endif

    syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    nErr = getState();
    if (nErr)
        nErr = ERR_NORESPONSE;
    return nErr;
}


void CRRCI::Disconnect()
{
    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}


int CRRCI::domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen, int nTimeout)
{
    int nErr = PluginOK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;

    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRRCI::domeCommand sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    if(!pszResult)
        return nErr;
    
    // read response
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE, nTimeout);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CRRCI::domeCommand ***** ERROR READING RESPONSE **** error = %d , response : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CRRCI::domeCommand response : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    strncpy(pszResult, szResp, nResultMaxLen);

    return nErr;

}

int CRRCI::readResponse(char *szRespBuffer, int nBufferLen, int nTimeout)
{
    int nErr = PluginOK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;

    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, nTimeout);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRRCI::readResponse] readFile error\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRRCI::readResponse] Timeout while waiting for response from controller\n", timestamp);
            fflush(Logfile);
#endif

            nErr = BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
    } while (*pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen );

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::readResponse] szRespBuffer = %s\n", timestamp, szRespBuffer);
    fflush(Logfile);
#endif
    
    if(ulTotalBytesRead>1)
        *(pszBufPtr-1) = 0; //remove the #

    return nErr;
}



int CRRCI::syncDome(double dAz, double dEl)
{

    m_dCurrentAzPosition = dAz;
    m_dCurrentElPosition = dEl;

    return PluginOK;
}

int CRRCI::parkDome()
{
    return PluginOK;
}

int CRRCI::unparkDome()
{
    return PluginOK;
}

int CRRCI::gotoAzimuth(double dNewAz)
{
    int nErr = PluginOK;


    m_dCurrentAzPosition = dNewAz;
    if(m_nRoofState == OPEN)
        m_dCurrentElPosition = 90.0;
    else
        m_dCurrentElPosition = 0.0;

    return nErr;
}

int CRRCI::openRoof()
{
    int nErr = PluginOK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CRRCI::openRoof] Opening roof\n", timestamp);
	fflush(Logfile);
#endif
    nErr = getState();
    if(nErr)
        return nErr;
    if(!m_bSafe)
        return ERR_CMDFAILED;
    
    nErr = domeCommand("open#", NULL, SERIAL_BUFFER_SIZE);
    m_RoofAction = OPENING;
    return nErr;
}

int CRRCI::closeRoof()
{
    int nErr = PluginOK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CRRCI::closeRoof] Closing roof\n", timestamp);
	fflush(Logfile);
#endif
    nErr = getState();
    if(nErr)
        return nErr;
    if(!m_bSafe)
        return ERR_CMDFAILED;

	// close B side first
	nErr = domeCommand("close#", NULL, SERIAL_BUFFER_SIZE);
    m_RoofAction = CLOSING;

    return nErr;
}

int CRRCI::findHome()
{
	int nErr = PluginOK;

	if(!m_bIsConnected)
		return NOT_CONNECTED;

	return nErr;
}

int CRRCI::isGoToComplete(bool &bComplete)
{
    int nErr = PluginOK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    bComplete = true;

    return nErr;
}

int CRRCI::isOpenComplete(bool &bComplete)
{
    int nErr = PluginOK;

    bComplete = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    nErr = getState();
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::isOpenComplete] error getting state   = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif
        return nErr;
    }

    if(m_RoofAction != OPENING) {
        if(m_nRoofState == OPEN)
            bComplete = true;
        return nErr;
    }
    
    if(m_nRoofState == OPEN) {
        abortMove();
        m_RoofAction = IDLE;
        bComplete = true;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::isOpenComplete] State : m_nRoofState  = %d\n", timestamp, m_nRoofState);
    fprintf(Logfile, "[%s] [CRRCI::isOpenComplete] State : bComplete     = %s\n", timestamp, bComplete?"Yes":"No");
    fflush(Logfile);
#endif
    return nErr;
}

int CRRCI::isCloseComplete(bool &bComplete)
{
    int nErr = PluginOK;

    bComplete = false;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getState();
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::isCloseComplete] error getting state = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif
        return nErr;
    }

    if(m_RoofAction != CLOSING) {
        if(m_nRoofState == CLOSED)
            bComplete = true;
        return nErr;
    }
    if(m_nRoofState == CLOSED) {
        abortMove();
        m_RoofAction = IDLE;
        bComplete = true;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::isCloseComplete] State : m_nRoofState = %d\n", timestamp, m_nRoofState);
    fprintf(Logfile, "[%s] [CRRCI::isCloseComplete] State : bComplete    = %s\n", timestamp, bComplete?"Yes":"No");
    fflush(Logfile);
#endif

    return nErr;
}


int CRRCI::isParkComplete(bool &bComplete)
{
    int nErr = PluginOK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    bComplete = true;
    return nErr;
}

int CRRCI::isUnparkComplete(bool &bComplete)
{
    int nErr = PluginOK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    bComplete = true;

    return nErr;
}

int CRRCI::isFindHomeComplete(bool &bComplete)
{
    int nErr = PluginOK;

    bComplete = true;

    return nErr;

}


int CRRCI::abortMove()
{
    int nErrx = PluginOK;
    int nErry = PluginOK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::abortMove] aborting roof move\n", timestamp);
    fflush(Logfile);
#endif

    nErrx = domeCommand("x#", NULL, SERIAL_BUFFER_SIZE);
    if(nErrx) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRRCI::abortMove] Error on 'x#' : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
    }
    nErry = domeCommand("y#", NULL, SERIAL_BUFFER_SIZE);
    if(nErry) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRRCI::abortMove] Error on 'y#' : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
    }

    return (nErrx | nErry);
}

#pragma mark - Getter / Setter


double CRRCI::getCurrentAz()
{
    return m_dCurrentAzPosition;
}

double CRRCI::getCurrentEl()
{

    return m_dCurrentElPosition;
}

int CRRCI::getState()
{
    int nErr = PluginOK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CRRCI::getState] Getting roof state\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("get#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nErr = parseFields(szResp, vFields, ',');
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRRCI::getState] Error parsing response : %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        m_bSafe = false;
        m_nRoofState = UNKNOWN;
    }


    if(vFields.size()>2) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRRCI::getState] vFields[0] : '%s'\n", timestamp, vFields[0].c_str());
        fprintf(Logfile, "[%s] [CRRCI::getState] vFields[1] : '%s'\n", timestamp, vFields[1].c_str());
        fprintf(Logfile, "[%s] [CRRCI::getState] vFields[2] : '%s'\n", timestamp, vFields[2].c_str());
        fflush(Logfile);
#endif
        // check roof state
        if (vFields[2].find("not_moving_o") !=-1) {
            if(vFields[0].find("opened") !=-1)
                m_nRoofState = OPEN;
            else
                m_nRoofState = UNKNOWN;
        }
        else if (vFields[2].find("not_moving_c") !=-1) {
            if(vFields[0].find("closed") !=-1)
                m_nRoofState = CLOSED;
            else
                m_nRoofState = UNKNOWN;
        }
        else if(vFields[2].find("moving") !=-1) {
            m_nRoofState = MOVING;
        }
        else if(vFields[2].find("unknown") !=-1) {
            m_nRoofState = UNKNOWN;
        }
        
        // check safe state
        if(vFields[1].find("unsafe") !=-1) {
            m_bSafe = false;
        }
        else if(vFields[1].find("safe") !=-1) {
            m_bSafe = true;
        }
    }


#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRRCI::getState] m_nRoofState (0=OPEN, 1=MOVING, 2=CLOSED , 3=UNKNOWN) : %d\n", timestamp, m_nRoofState);
        fprintf(Logfile, "[%s] [CRRCI::getState] m_bSafe                                               : %s\n", timestamp, m_bSafe?"Yes":"No");
        fflush(Logfile);
#endif

    return nErr;
}

int CRRCI::parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PluginOK;
    std::string sSegment;
    if(!pszResp) {
        return ERR_CMDFAILED;
    }

    if(!strlen(pszResp)) {
        return ERR_CMDFAILED;
    }

    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRRCI::parseFields] sSegment : %s\n", timestamp, sSegment.c_str());
        fflush(Logfile);
#endif
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}


std::string& CRRCI::trim(std::string &str, const std::string& filter )
{
    return ltrim(rtrim(str, filter), filter);
}

std::string& CRRCI::ltrim(std::string& str, const std::string& filter)
{
    str.erase(0, str.find_first_not_of(filter));
    return str;
}

std::string& CRRCI::rtrim(std::string& str, const std::string& filter)
{
    str.erase(str.find_last_not_of(filter) + 1);
    return str;
}

std::string CRRCI::findField(std::vector<std::string> &svFields, const std::string& token)
{
    for(int i=0; i<svFields.size(); i++){
        if(svFields[i].find(token)!= -1) {
            return svFields[i];
        }
    }
    return std::string();
}


