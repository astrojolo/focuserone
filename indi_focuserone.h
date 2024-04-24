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

#ifndef ASTROLINK4_H
#define ASTROLINK4_H

#include <string>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <memory>
#include <regex>
#include <cstring>
#include <map>
#include <sstream>

#include <defaultdevice.h>
#include <indifocuserinterface.h>
#include <indiweatherinterface.h>
#include <connectionplugins/connectionserial.h>

#define Q_STEPPER_POS		1
#define Q_STEPS_TO_GO		2
#define Q_SENS1_TYPE		3
#define Q_SENS1_TEMP		4
#define Q_SENS1_HUM			5
#define Q_SENS1_DEW			6
#define Q_SENS2_TYPE		7
#define Q_SENS2_TEMP		8
#define Q_COMP_DIFF			9

#define U_CURRENT           1
#define U_STOP_CURRENT      2
#define U_SPEED				3
#define U_STEPPER_MODE		4
#define U_MAX_POS			5
#define U_REVERSED			6
#define U_STEPSIZE			7
#define U_COMPSENS			8
#define U_COMP_STEPS		8
#define U_COMP_CYCLE		9
#define U_COMP_TRGR			10
#define U_COMP_AUTO			11
#define U_CONTRAST_H        12
#define U_CONTRAST_L        13
#define U_CONTRAST_DELAY    14
#define U_SHOWSPLASH        15
#define U_DISABLE_OLED      16
#define U_STEPS_PER_DEG     17
#define U_MODE              18
#define U_ROTATOR_OFFSET    19




namespace Connection
{
class Serial;
}

class FocuserOne : public INDI::DefaultDevice, public INDI::FocuserInterface, public INDI::WeatherInterface
{

public:
    FocuserOne();
    virtual bool initProperties();
    virtual bool updateProperties();
	
    virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n);
    virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);
    virtual bool ISNewText(const char * dev, const char * name, char * texts[], char * names[], int n);
	
protected:
    virtual const char *getDefaultName();
    virtual void TimerHit();
    virtual bool saveConfigItems(FILE *fp);
    virtual bool sendCommand(const char * cmd, char * res);

    // Focuser Overrides
    virtual IPState MoveAbsFocuser(uint32_t targetTicks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    virtual bool AbortFocuser() override;
    virtual bool ReverseFocuser(bool enabled) override;
    virtual bool SyncFocuser(uint32_t ticks) override;

    virtual bool SetFocuserBacklash(int32_t steps) override;
    virtual bool SetFocuserBacklashEnabled(bool enabled) override;
    virtual bool SetFocuserMaxPosition(uint32_t ticks) override;

    // Weather Overrides
    virtual IPState updateWeather() override
    {
        return IPS_OK;
    }

	
private:
    virtual bool Handshake();
    int PortFD = -1;
    Connection::Serial *serialConnection { nullptr };
    bool updateSettings(const char * getCom, const char * setCom, int index, const char * value);
    bool updateSettings(const char * getCom, const char * setCom, std::map<int, std::string> values);
    std::vector<std::string> split(const std::string &input, const std::string &regex);
    std::string doubleToStr(double val);
    bool sensorRead();
    int32_t calculateBacklash(uint32_t targetTicks);
    char stopChar { 0xA };	// new line
    bool backlashEnabled = false;
    int32_t backlashSteps = 0;
    bool requireBacklashReturn = false;

    INumber FocusPosMMN[1];
    INumberVectorProperty FocusPosMMNP;

    INumber CompensationValueN[1];
    INumberVectorProperty CompensationValueNP;
    ISwitch CompensateNowS[1];
    ISwitchVectorProperty CompensateNowSP;

    INumber FocuserSettingsN[5];
    INumberVectorProperty FocuserSettingsNP;
    enum
    {
        FS_SPEED, FS_STEP_SIZE, FS_CURRENT, FS_COMPENSATION, FS_COMP_THRESHOLD
    };
    ISwitch FocuserModeS[4];
    ISwitchVectorProperty FocuserModeSP;
    enum
    {
      FS_MODE_UNI, FS_MODE_BI, FS_MODE_MICRO_L, FS_MODE_MICRO_H
    };
    ISwitch FocuserHoldS[2];
    ISwitchVectorProperty FocuserHoldSP;
    enum
    {
      FS_HOLD_ON, FS_HOLD_OFF
    };

    ISwitch FocuserCompModeS[2];
    ISwitchVectorProperty FocuserCompModeSP;
    enum
    {
    FS_COMP_AUTO, FS_COMP_MANUAL
    };
    
    ISwitch FocuserManualS[2];
    ISwitchVectorProperty FocuserManualSP;
    enum
    {
    FS_MANUAL_ON, FS_MANUAL_OFF
    };

    ISwitch BuzzerS[1];
    ISwitchVectorProperty BuzzerSP;
    
    static constexpr const char *ENVIRONMENT_TAB {"Environment"};
    static constexpr const char *SETTINGS_TAB {"Settings"};
};

#endif
