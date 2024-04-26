/*******************************************************************************
 Copyright(c) 2019 astrojolo.com
 .
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/
#include "indi_focuserone.h"

#include "indicom.h"

#define VERSION_MAJOR 1
#define VERSION_MINOR 0

#define ASTROLINK4_LEN      100
#define ASTROLINK4_TIMEOUT  3

#define POLLTIME 500

//////////////////////////////////////////////////////////////////////
/// Delegates
//////////////////////////////////////////////////////////////////////
std::unique_ptr<FocuserOne> indiFocuserOne(new FocuserOne());

void ISGetProperties(const char *dev)
{
    indiFocuserOne->ISGetProperties(dev);
}
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    indiFocuserOne->ISNewSwitch(dev, name, states, names, num);
}
void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
    indiFocuserOne->ISNewText (dev, name, texts, names, num);
}
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    indiFocuserOne->ISNewNumber(dev, name, values, names, num);
}
void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int num)
{
    indiFocuserOne->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, num);
}
void ISSnoopDevice (XMLEle *root)
{
    indiFocuserOne->ISSnoopDevice(root);
}

//////////////////////////////////////////////////////////////////////
///Constructor
//////////////////////////////////////////////////////////////////////
FocuserOne::FocuserOne() : FI(this), WI(this)
{
    setVersion(VERSION_MAJOR,VERSION_MINOR);
}

const char * FocuserOne::getDefaultName()
{
    return (char *)"FocuserOne";
}

//////////////////////////////////////////////////////////////////////
/// Communication
//////////////////////////////////////////////////////////////////////
bool FocuserOne::Handshake()
{
    PortFD = serialConnection->getPortFD();

    char res[ASTROLINK4_LEN] = {0};
    if(sendCommand("#", res))
    {
        if(strncmp(res, "#:FocuserOne", 12) != 0)
        {
            LOG_ERROR("Device not recognized.");
            return false;
        }
        else
        {
            SetTimer(POLLTIME);
            return true;
        }
    }
    return false;
}

void FocuserOne::TimerHit()
{
    if(isConnected())
    {
        sensorRead();
        SetTimer(POLLTIME);
    }
}

//////////////////////////////////////////////////////////////////////
/// Overrides
//////////////////////////////////////////////////////////////////////
bool FocuserOne::initProperties()
{
    INDI::DefaultDevice::initProperties();

    setDriverInterface(AUX_INTERFACE | FOCUSER_INTERFACE | WEATHER_INTERFACE);

    FI::SetCapability(FOCUSER_CAN_ABS_MOVE |
                      FOCUSER_CAN_REL_MOVE |
                      FOCUSER_CAN_REVERSE  |
                      FOCUSER_CAN_SYNC     |
                      FOCUSER_CAN_ABORT    |
                      FOCUSER_HAS_BACKLASH);

    FI::initProperties(FOCUS_TAB);
    WI::initProperties(ENVIRONMENT_TAB, ENVIRONMENT_TAB);

    addDebugControl();
    addSimulationControl();
    addConfigurationControl();

    serialConnection = new Connection::Serial(this);
    serialConnection->registerHandshake([&]()
    {
        return Handshake();
    });
    registerConnection(serialConnection);

    serialConnection->setDefaultPort("/dev/ttyUSB0");
    serialConnection->setDefaultBaudRate(serialConnection->B_38400);

    // focuser settings
    IUFillNumber(&FocuserSettingsN[FS_SPEED], "FS_SPEED", "Speed [pps]", "%.0f", 0, 4000, 50, 250);
    IUFillNumber(&FocuserSettingsN[FS_STEP_SIZE], "FS_STEP_SIZE", "Step size [um]", "%.2f", 0, 100, 0.1, 5.0);
    IUFillNumber(&FocuserSettingsN[FS_CURRENT], "FS_CURRENT", "Stepper current [mA]", "%.0f", 100, 1500, 100, 400);
    IUFillNumber(&FocuserSettingsN[FS_COMPENSATION], "FS_COMPENSATION", "Compensation [steps/C]", "%.2f", -1000, 1000, 1, 0);
    IUFillNumber(&FocuserSettingsN[FS_COMP_THRESHOLD], "FS_COMP_THRESHOLD", "Compensation threshold [steps]", "%.0f", 1, 1000, 10, 10);
    IUFillNumberVector(&FocuserSettingsNP, FocuserSettingsN, 5, getDeviceName(), "FOCUSER_SETTINGS", "Focuser settings", SETTINGS_TAB, IP_RW, 60, IPS_IDLE);

    IUFillSwitch(&FocuserHoldS[FS_HOLD_OFF], "FS_HOLD_OFF", "OFF", ISS_OFF);
    IUFillSwitch(&FocuserHoldS[FS_HOLD_ON], "FS_HOLD_ON", "ON", ISS_ON);
    IUFillSwitchVector(&FocuserHoldSP, FocuserHoldS, 2, getDeviceName(), "MOTOR_HOLD", "Stepper holding torque", SETTINGS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&FocuserModeS[FS_MODE_UNI], "FS_MODE_UNI", "Unipolar", ISS_ON);
    IUFillSwitch(&FocuserModeS[FS_MODE_BI], "FS_MODE_BI", "Bipolar", ISS_OFF);
    IUFillSwitch(&FocuserModeS[FS_MODE_MICRO_L], "FS_MODE_MICRO_L", "Microstep 1/8", ISS_OFF);
    IUFillSwitch(&FocuserModeS[FS_MODE_MICRO_H], "FS_MODE_MICRO_H", "Microstep 1/32", ISS_OFF);
    IUFillSwitchVector(&FocuserModeSP, FocuserModeS, 4, getDeviceName(), "FOCUSER_MODE", "Focuser mode", SETTINGS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&FocuserCompModeS[FS_COMP_AUTO], "FS_COMP_AUTO", "AUTO", ISS_OFF);
    IUFillSwitch(&FocuserCompModeS[FS_COMP_MANUAL], "FS_COMP_MANUAL", "MANUAL", ISS_ON);
    IUFillSwitchVector(&FocuserCompModeSP, FocuserCompModeS, 2, getDeviceName(), "COMP_MODE", "Compensation mode", SETTINGS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    IUFillSwitch(&FocuserManualS[FS_MANUAL_ON], "FS_MANUAL_ON", "ON", ISS_ON);
    IUFillSwitch(&FocuserManualS[FS_MANUAL_OFF], "FS_MANUAL_OFF", "OFF", ISS_OFF);
    IUFillSwitchVector(&FocuserManualSP, FocuserManualS, 2, getDeviceName(), "MANUAL_CONTROLLER", "Hand controller", SETTINGS_TAB, IP_RW, ISR_1OFMANY, 60, IPS_IDLE);

    // other settings
    IUFillSwitch(&BuzzerS[0], "BUZZER", "Buzzer", ISS_OFF);
    IUFillSwitchVector(&BuzzerSP, BuzzerS, 1, getDeviceName(), "BUZZER", "ONOFF", SETTINGS_TAB, IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

    // focuser compensation
    IUFillNumber(&CompensationValueN[0], "COMP_VALUE", "Compensation steps", "%.0f", -10000, 10000, 1, 0);
    IUFillNumberVector(&CompensationValueNP, CompensationValueN, 1, getDeviceName(), "COMP_STEPS", "Compensation steps", FOCUS_TAB, IP_RO, 60, IPS_IDLE);

    IUFillSwitch(&CompensateNowS[0], "COMP_NOW", "Compensate now", ISS_OFF);
    IUFillSwitchVector(&CompensateNowSP, CompensateNowS, 1, getDeviceName(), "COMP_NOW", "Compensate now", FOCUS_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    IUFillNumber(&FocusPosMMN[0], "FOC_POS_MM", "Position [mm]", "%.3f", 0.0, 200.0, 0.001, 0.0);
    IUFillNumberVector(&FocusPosMMNP, FocusPosMMN, 1, getDeviceName(), "FOC_POS_MM", "Position [mm]", FOCUS_TAB, IP_RO, 60, IPS_IDLE);

    // Environment Group
    addParameter("WEATHER_TEMPERATURE", "Temperature (C)", -15, 35, 15);
    addParameter("WEATHER_HUMIDITY", "Humidity %", 0, 100, 15);
    addParameter("WEATHER_DEWPOINT", "Dew Point (C)", 0, 100, 15);

    return true;
}

bool FocuserOne::updateProperties()
{
    // Call parent update properties first
    INDI::DefaultDevice::updateProperties();

    if (isConnected())
    {
    	defineProperty(&FocusPosMMNP);
        FI::updateProperties();
        WI::updateProperties();
        defineProperty(&FocuserSettingsNP);
        defineProperty(&FocuserModeSP);
        defineProperty(&FocuserCompModeSP);
        defineProperty(&FocuserHoldSP);
        defineProperty(&FocuserManualSP);
        defineProperty(&CompensationValueNP);
        defineProperty(&CompensateNowSP);
        defineProperty(&BuzzerSP);
    }
    else
    {
        deleteProperty(FocuserSettingsNP.name);
        deleteProperty(FocuserModeSP.name);
        deleteProperty(CompensateNowSP.name);
        deleteProperty(CompensationValueNP.name);
        deleteProperty(BuzzerSP.name);
        deleteProperty(FocuserCompModeSP.name);
        deleteProperty(FocuserHoldSP.name);
        deleteProperty(FocuserManualSP.name);
        deleteProperty(FocusPosMMNP.name);
        FI::updateProperties();
        WI::updateProperties();
    }

    return true;
}

bool FocuserOne::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()))
    {
        char cmd[ASTROLINK4_LEN] = {0};
        char res[ASTROLINK4_LEN] = {0};
        
        
        // Focuser settings
        if(!strcmp(name, FocuserSettingsNP .name))
        {
        	bool allOk = true;
        	std::map<int, std::string> updates;
            updates[U_SPEED] = doubleToStr(values[FS_SPEED]);
            //updates[U_ACC] = doubleToStr(values[FS_SPEED] * 2.0);
            updates[U_STEPSIZE] = doubleToStr(values[FS_STEP_SIZE] * 100.0);
            updates[U_COMP_CYCLE] = "30";  // cycle [s]
            updates[U_COMP_STEPS] = doubleToStr(values[FS_COMPENSATION] * 100.0);
            //updates[U_COMP_SENSR] = "0";   // sensor
            updates[U_COMP_TRGR] = doubleToStr(values[FS_COMP_THRESHOLD]);
            updates[U_CURRENT] = doubleToStr(values[FS_CURRENT]);
            allOk = allOk && updateSettings("u", "U", updates);
        	if(allOk)
        	{
                FocuserSettingsNP.s = IPS_BUSY;
                IUUpdateNumber(&FocuserSettingsNP, values, names, n);
                IDSetNumber(&FocuserSettingsNP, nullptr);
                LOG_INFO(values[FS_COMPENSATION] > 0 ? "Temperature compensation is enabled." : "Temperature compensation is disabled.");
                return true;
        	}
            FocuserSettingsNP.s = IPS_ALERT;
            return true;
        }

        if (strstr(name, "FOCUS_"))
            return FI::processNumber(dev, name, values, names, n);
        if (strstr(name, "WEATHER_"))
            return WI::processNumber(dev, name, values, names, n);
    }

	return INDI::DefaultDevice::ISNewNumber(dev,name,values,names,n);
}


bool FocuserOne::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()))
    {
        char cmd[ASTROLINK4_LEN] = {0};
        char res[ASTROLINK4_LEN] = {0};

        // compensate now
        if(!strcmp(name, CompensateNowSP.name))
        {
            sprintf(cmd, "S:%d", static_cast<uint16_t>(FocuserSettingsN[FS_COMP_THRESHOLD].value));
            bool allOk = sendCommand(cmd, res);
            CompensateNowSP.s = allOk ? IPS_BUSY : IPS_ALERT;
            if(allOk)
                IUUpdateSwitch(&CompensateNowSP, states, names, n);

            IDSetSwitch(&CompensateNowSP, nullptr);
            return true;
        }

        // Buzzer
        if(!strcmp(name, BuzzerSP.name))
        {
            if(updateSettings("j", "J", 1, (states[0] == ISS_ON) ? "1" : "0"))
        	{
                BuzzerSP.s = IPS_BUSY;
                IUUpdateSwitch(&BuzzerSP, states, names, n);
                IDSetSwitch(&BuzzerSP, nullptr);
                return true;
        	}
            BuzzerSP.s = IPS_ALERT;
            return true;
        }

        // Manual mode
        if(!strcmp(name, FocuserManualSP.name))
        {
            sprintf(cmd, "F:%s", (strcmp(FocuserManualS[0].name, names[0])) ? "0" : "1");
            if(sendCommand(cmd, res))
        	{
            	FocuserManualSP.s = IPS_BUSY;
                IUUpdateSwitch(&FocuserManualSP, states, names, n);
                IDSetSwitch(&FocuserManualSP, nullptr);
                return true;
        	}
            FocuserManualSP.s = IPS_ALERT;
            return true;
        }
        
        // Focuser Mode
        if(!strcmp(name, FocuserModeSP.name))
        {
            std::string value = "0";
            if(!strcmp(FocuserModeS[FS_MODE_UNI].name, names[0])) value = "0";
            if(!strcmp(FocuserModeS[FS_MODE_BI].name, names[0])) value = "1";
            if(!strcmp(FocuserModeS[FS_MODE_MICRO_L].name, names[0])) value = "2";
            if(!strcmp(FocuserModeS[FS_MODE_MICRO_H].name, names[0])) value = "3";
            if(updateSettings("u", "U", U_STEPPER_MODE, value.c_str()))
        	{
                FocuserModeSP.s = IPS_BUSY;
                IUUpdateSwitch(&FocuserModeSP, states, names, n);
                IDSetSwitch(&FocuserModeSP, nullptr);
                return true;
        	}
            FocuserModeSP.s = IPS_ALERT;
            return true;
        }

        // Focuser compensation mode
        if(!strcmp(name, FocuserCompModeSP.name))
        {
        	std::string value = "0";
        	if(!strcmp(FocuserCompModeS[FS_COMP_AUTO].name, names[0])) value = "1";
        	if(updateSettings("u", "U", U_COMP_AUTO, value.c_str()))
        	{
        		FocuserCompModeSP.s = IPS_BUSY;
                IUUpdateSwitch(&FocuserCompModeSP, states, names, n);
                IDSetSwitch(&FocuserCompModeSP, nullptr);
                return true;
        	}
        	FocuserCompModeSP.s = IPS_ALERT;
            return true;
        }

        // Focuser stepper holding torque mode
        if(!strcmp(name, FocuserHoldSP.name))
        {
            std::string value = "3";
            if(!strcmp(FocuserHoldS[FS_HOLD_ON].name, names[0])) value = "2";
            if(updateSettings("u", "U", U_STOP_CURRENT, value.c_str()))
            {
                FocuserHoldSP.s = IPS_BUSY;
                IUUpdateSwitch(&FocuserHoldSP, states, names, n);
                IDSetSwitch(&FocuserHoldSP, nullptr);
                return true;
            }
            FocuserHoldSP.s = IPS_ALERT;
            return true;
        }

        if (strstr(name, "FOCUS_")) 
            return FI::processSwitch(dev, name, states, names, n);
        if (strstr(name, "WEATHER_")) 
            return WI::processSwitch(dev, name, states, names, n);
	}

	return INDI::DefaultDevice::ISNewSwitch (dev, name, states, names, n);
}

bool FocuserOne::ISNewText(const char * dev, const char * name, char * texts[], char * names[], int n)
{
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}


bool FocuserOne::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    FI::saveConfigItems(fp);

    return true;
}


//////////////////////////////////////////////////////////////////////
/// Focuser interface
//////////////////////////////////////////////////////////////////////
IPState FocuserOne::MoveAbsFocuser(uint32_t targetTicks)
{
	int32_t backlash = 0;
	if(backlashEnabled)
	{
		if((targetTicks > FocusAbsPosN[0].value) == (backlashSteps > 0))
		{
			if((targetTicks + backlash) < 0 || (targetTicks + backlash) > FocusMaxPosN[0].value)
			{
				backlash = 0;
			}
			else
			{
				backlash = backlashSteps;
				requireBacklashReturn = true;
			}
		}
	}
    char cmd[ASTROLINK4_LEN] = {0}, res[ASTROLINK4_LEN] = {0};
    snprintf(cmd, ASTROLINK4_LEN, "R:0:%u", targetTicks + backlash);
    return (sendCommand(cmd, res)) ? IPS_BUSY : IPS_ALERT;
}

IPState FocuserOne::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    return MoveAbsFocuser(dir == FOCUS_INWARD ? FocusAbsPosN[0].value - ticks : FocusAbsPosN[0].value + ticks);
}

bool FocuserOne::AbortFocuser()
{
    char res[ASTROLINK4_LEN] = {0};
    return (sendCommand("H", res));
}

bool FocuserOne::ReverseFocuser(bool enabled)
{
    return updateSettings("u", "U", U_REVERSED, (enabled) ? "1" : "0");
}

bool FocuserOne::SyncFocuser(uint32_t ticks)
{
    char cmd[ASTROLINK4_LEN] = {0}, res[ASTROLINK4_LEN] = {0};
    snprintf(cmd, ASTROLINK4_LEN, "P:%u", ticks);
    return sendCommand(cmd, res);
}

bool FocuserOne::SetFocuserMaxPosition(uint32_t ticks)
{
    if(updateSettings("u", "U", U_MAX_POS, std::to_string(ticks).c_str()))
    {
        FocuserSettingsNP.s = IPS_BUSY;
        return true;
    }
    else
    {
        return false;
    }
}

bool FocuserOne::SetFocuserBacklash(int32_t steps)
{
    backlashSteps = steps;
    return true;
}

bool FocuserOne::SetFocuserBacklashEnabled(bool enabled)
{
    backlashEnabled = enabled;
    return true;
}

//////////////////////////////////////////////////////////////////////
/// Serial commands
//////////////////////////////////////////////////////////////////////
bool FocuserOne::sendCommand(const char * cmd, char * res)
{
    int nbytes_read = 0, nbytes_written = 0, tty_rc = 0;
    char command[ASTROLINK4_LEN];

    if(isSimulation())
    {
        if(strcmp(cmd, "#") == 0) sprintf(res, "%s\n", "#:FocuserOne");
        if(strcmp(cmd, "q") == 0) sprintf(res, "%s\n", "q:1234:0:1.47:1:2.12:45.1:-12.81:1:-25.22:45:0:0:0:1:12.1:5.0:1.12:13.41:0:34:0:0");
        if(strcmp(cmd, "p") == 0) sprintf(res, "%s\n", "p:1234");
        if(strcmp(cmd, "i") == 0) sprintf(res, "%s\n", "i:0");
        if(strcmp(cmd, "n") == 0) sprintf(res, "%s\n", "n:1077:14.0:10.0:100");
        if(strcmp(cmd, "e") == 0) sprintf(res, "%s\n", "e:30:1200:1:0:20");
        if(strcmp(cmd, "u") == 0) sprintf(res, "%s\n", "u:25000:220:0:100:440:0:0:1:257:0:0:0:0:0:1:0:0");
        if(strncmp(cmd, "R", 1) == 0) sprintf(res, "%s\n", "R:");
        if(strncmp(cmd, "C", 1) == 0) sprintf(res, "%s\n", "C:");
        if(strncmp(cmd, "B", 1) == 0) sprintf(res, "%s\n", "B:");
        if(strncmp(cmd, "H", 1) == 0) sprintf(res, "%s\n", "H:");
        if(strncmp(cmd, "P", 1) == 0) sprintf(res, "%s\n", "P:");
        if(strncmp(cmd, "U", 1) == 0) sprintf(res, "%s\n", "U:");
        if(strncmp(cmd, "S", 1) == 0) sprintf(res, "%s\n", "S:");
        if(strncmp(cmd, "G", 1) == 0) sprintf(res, "%s\n", "G:");
        if(strncmp(cmd, "K", 1) == 0) sprintf(res, "%s\n", "K:");
        if(strncmp(cmd, "N", 1) == 0) sprintf(res, "%s\n", "N:");
        if(strncmp(cmd, "E", 1) == 0) sprintf(res, "%s\n", "E:");
    }
    else
    {
        tcflush(PortFD, TCIOFLUSH);
        sprintf(command, "%s\n", cmd);
        LOGF_DEBUG("CMD %s", command);
        if ( (tty_rc = tty_write_string(PortFD, command, &nbytes_written)) != TTY_OK)
            return false;

        if (!res)
        {
            tcflush(PortFD, TCIOFLUSH);
            return true;
        }

        if ( (tty_rc = tty_nread_section(PortFD, res, ASTROLINK4_LEN, stopChar, ASTROLINK4_TIMEOUT, &nbytes_read)) != TTY_OK || nbytes_read == 1)
            return false;

        tcflush(PortFD, TCIOFLUSH);
        res[nbytes_read - 1] = '\0';
        LOGF_DEBUG("RES %s", res);

        if (tty_rc != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(tty_rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial error: %s", errorMessage);
            return false;
        }
    }
    
    return (cmd[0] == res[0]);
}

//////////////////////////////////////////////////////////////////////
/// Sensors
//////////////////////////////////////////////////////////////////////
bool FocuserOne::sensorRead()
{
    char res[ASTROLINK4_LEN] = {0};
    LOGF_DEBUG("Answer length %i", ASTROLINK4_LEN);
    if (sendCommand("q", res))
    {
        LOGF_DEBUG("Response %s", res);
        std::vector<std::string> result = split(res, ":");
        float focuserPosition = std::stod(result[Q_STEPPER_POS]);
        FocusAbsPosN[0].value = focuserPosition;
        FocusPosMMN[0].value = focuserPosition * FocuserSettingsN[FS_STEP_SIZE].value / 1000.0;
        LOGF_DEBUG("Foc pos %f", focuserPosition);
        float stepsToGo = std::stod(result[Q_STEPS_TO_GO]);
        LOGF_DEBUG("Foc to go %f", stepsToGo);

        if(stepsToGo == 0)
        {
        	if(requireBacklashReturn)
        	{
        		requireBacklashReturn = false;
        		MoveAbsFocuser(focuserPosition - backlashSteps);
        	}
            FocusAbsPosNP.s = FocusRelPosNP.s = FocusPosMMNP.s = IPS_OK;
            IDSetNumber(&FocusRelPosNP, nullptr);
        }
        else
        {
            FocusAbsPosNP.s = FocusRelPosNP.s = FocusPosMMNP.s = IPS_BUSY;
        }
        IDSetNumber(&FocusPosMMNP, nullptr);
        IDSetNumber(&FocusAbsPosNP, nullptr);

        LOGF_DEBUG("%s", "Before if");
        if(result.size() > 5)
        {
            LOGF_DEBUG("%s", "After if");
            if(std::stod(result[Q_SENS1_TYPE]) > 0)
            {
                setParameterValue("WEATHER_TEMPERATURE", std::stod(result[Q_SENS1_TEMP]));
                setParameterValue("WEATHER_HUMIDITY", std::stod(result[Q_SENS1_HUM]));
                setParameterValue("WEATHER_DEWPOINT", std::stod(result[Q_SENS1_DEW]));
            }
            LOGF_DEBUG("%s", "After sens type");
            CompensationValueN[0].value = std::stod(result[Q_COMP_DIFF]);
            CompensateNowSP.s = CompensationValueNP.s = (CompensationValueN[0].value > 0) ? IPS_OK : IPS_IDLE;
            CompensateNowS[0].s = (CompensationValueN[0].value != 0) ? ISS_OFF : ISS_ON;
            IDSetNumber(&CompensationValueNP, nullptr);
            IDSetSwitch(&CompensateNowSP, nullptr);
        }
    }
    LOGF_DEBUG("%s", "End if");
    // update settings data if was changed
    if(FocuserSettingsNP.s != IPS_OK || FocuserModeSP.s != IPS_OK || BuzzerSP.s != IPS_OK || FocuserCompModeSP.s != IPS_OK || FocuserHoldSP.s != IPS_OK)
    {
        if (sendCommand("u", res))
        {
            std::vector<std::string> result = split(res, ":");
            
            FocuserModeS[FS_MODE_UNI].s = FocuserModeS[FS_MODE_BI].s = FocuserModeS[FS_MODE_MICRO_L].s = FocuserModeS[FS_MODE_MICRO_H].s = ISS_OFF;
            if(!strcmp("0", result[U_STEPPER_MODE].c_str())) FocuserModeS[FS_MODE_UNI].s = ISS_ON;
            if(!strcmp("1", result[U_STEPPER_MODE].c_str())) FocuserModeS[FS_MODE_BI].s = ISS_ON;
            if(!strcmp("2", result[U_STEPPER_MODE].c_str())) FocuserModeS[FS_MODE_MICRO_L].s = ISS_ON;
            if(!strcmp("3", result[U_STEPPER_MODE].c_str())) FocuserModeS[FS_MODE_MICRO_H].s = ISS_ON;
            FocuserModeSP.s = IPS_OK;
            IDSetSwitch(&FocuserModeSP, nullptr);
            
            FocuserSettingsN[FS_SPEED].value = std::stod(result[U_SPEED]);
            FocuserSettingsN[FS_STEP_SIZE].value = std::stod(result[U_STEPSIZE]) / 100.0;
            FocusMaxPosN[0].value = std::stod(result[U_MAX_POS]);
            FocuserSettingsNP.s = IPS_OK;
            IDSetNumber(&FocuserSettingsNP, nullptr);
            IDSetNumber(&FocusMaxPosNP, nullptr);

            FocuserSettingsN[FS_CURRENT].value = std::stod(result[U_CURRENT]);
            FocuserHoldS[FS_HOLD_OFF].s = (std::stod(result[U_STOP_CURRENT]) == 3) ? ISS_ON : ISS_OFF;
            FocuserHoldS[FS_HOLD_ON].s = (std::stod(result[U_STOP_CURRENT]) < 3) ? ISS_ON : ISS_OFF;
            FocuserHoldSP.s = IPS_OK;
            IDSetSwitch(&FocuserHoldSP, nullptr);

            FocuserSettingsN[FS_COMPENSATION].value = std::stod(result[U_COMP_STEPS]) / 100.0;
            FocuserSettingsN[FS_COMP_THRESHOLD].value = std::stod(result[U_COMP_TRGR]);
            FocuserSettingsNP.s = IPS_OK;
            IDSetNumber(&FocuserSettingsNP, nullptr);

            FocuserCompModeS[FS_COMP_MANUAL].s = (std::stod(result[U_COMP_AUTO]) == 0) ? ISS_ON : ISS_OFF;
            FocuserCompModeS[FS_COMP_AUTO].s = (std::stod(result[U_COMP_AUTO]) > 0) ? ISS_ON : ISS_OFF;
            FocuserCompModeSP.s = IPS_OK;
            IDSetSwitch(&FocuserCompModeSP, nullptr);            
        }

        if(sendCommand("j", res))
        {
            std::vector<std::string> result = split(res, ":");
            BuzzerS[0].s = (std::stod(result[1]) > 0) ? ISS_ON : ISS_OFF;
            BuzzerSP.s = IPS_OK;
            IDSetSwitch(&BuzzerSP, nullptr);
        }
/*
        if (sendCommand("e", res))
        {
            std::vector<std::string> result = split(res, ":");
            FocuserSettingsN[FS_COMPENSATION].value = std::stod(result[E_COMP_STEPS]) / 100.0;
            FocuserSettingsN[FS_COMP_THRESHOLD].value = std::stod(result[E_COMP_TRGR]);
            FocuserSettingsNP.s = IPS_OK;
            IDSetNumber(&FocuserSettingsNP, nullptr);

            FocuserCompModeS[FS_COMP_MANUAL].s = (std::stod(result[E_COMP_AUTO]) == 0) ? ISS_ON : ISS_OFF;
            FocuserCompModeS[FS_COMP_AUTO].s = (std::stod(result[E_COMP_AUTO]) > 0) ? ISS_ON : ISS_OFF;
            FocuserCompModeSP.s = IPS_OK;
            IDSetSwitch(&FocuserCompModeSP, nullptr);
        }

        if (sendCommand("z", res))
        {
            std::vector<std::string> result = split(res, ":");
            FocuserSettingsN[FS_CURRENT].value = std::stod(result[Z_CURRENT]);
            FocuserHoldS[FS_HOLD_OFF].s = (std::stod(result[Z_STOP_CURRENT]) == 3) ? ISS_ON : ISS_OFF;
            FocuserHoldS[FS_HOLD_ON].s = (std::stod(result[Z_STOP_CURRENT]) < 3) ? ISS_ON : ISS_OFF;
            FocuserHoldSP.s = IPS_OK;
            IDSetSwitch(&FocuserHoldSP, nullptr);
        }
        */
    }
/*
    if(FocuserManualSP.s != IPS_OK)
    {
        if (sendCommand("f", res))
        {
            std::vector<std::string> result = split(res, ":");
            FocuserManualS[FS_MANUAL_OFF].s = (std::stod(result[1]) == 0) ? ISS_ON : ISS_OFF;
            FocuserManualS[FS_MANUAL_ON].s = (std::stod(result[1]) > 0) ? ISS_ON : ISS_OFF;
            FocuserManualSP.s = IPS_OK;
            IDSetSwitch(&FocuserManualSP, nullptr);
        }
    }
*/
    return true;
}

//////////////////////////////////////////////////////////////////////
/// Helper functions
//////////////////////////////////////////////////////////////////////
std::vector<std::string> FocuserOne::split(const std::string &input, const std::string &regex)
{
    // passing -1 as the submatch index parameter performs splitting
    std::regex re(regex);
    std::sregex_token_iterator
    first{input.begin(), input.end(), re, -1},
          last;
    return {first, last};
}

std::string FocuserOne::doubleToStr(double val)
{
    char buf[10];
    sprintf(buf, "%.0f", val);
    return std::string(buf);
}

bool FocuserOne::updateSettings(const char * getCom, const char * setCom, int index, const char * value)
{
	std::map<int, std::string> values;
	values[index] = value;
    return updateSettings(getCom, setCom, values);
}

bool FocuserOne::updateSettings(const char * getCom, const char * setCom, std::map<int, std::string> values)
{
    char cmd[ASTROLINK4_LEN] = {0}, res[ASTROLINK4_LEN] = {0};
    snprintf(cmd, ASTROLINK4_LEN, "%s", getCom);
    if(sendCommand(cmd, res))
    {
        std::string concatSettings = "";
        std::vector<std::string> result = split(res, ":");
        if(result.size() >= values.size())
        {
            result[0] = setCom;
            for(std::map<int, std::string>::iterator it = values.begin(); it != values.end(); ++it)
            	result[it->first] = it->second;

            for (const auto &piece : result) concatSettings += piece + ":";
            snprintf(cmd, ASTROLINK4_LEN, "%s", concatSettings.c_str());
            if(sendCommand(cmd, res)) return true;
        }
    }
	return false;
}
