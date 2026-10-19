/*
    LX200 GPS
    Copyright (C) 2003 Jasem Mutlaq (mutlaqja@ikarustech.com)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "lx200gps.h"

#include "lx200driver.h"
#include "indicom.h"

#include <libnova/julian_day.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <termios.h>
#include <unistd.h>

#define GPS_TAB "Extended GPS Features"

// Limit of the custom axis rates, arcsec/s
#define LX200GPS_MAX_CUSTOM_RATE 16384.0

LX200GPS::LX200GPS() : LX200Autostar()
{
    MaxReticleFlashRate = 9;
    SetTelescopeCapability(GetTelescopeCapability() | TELESCOPE_CAN_CONTROL_TRACK | TELESCOPE_HAS_TRACK_RATE, 4);
}

const char *LX200GPS::getDefaultName()
{
    return "LX200 GPS";
}

bool LX200GPS::initProperties()
{
    LX200Autostar::initProperties();

    IUFillSwitch(&GPSPowerS[0], "On", "", ISS_OFF);
    IUFillSwitch(&GPSPowerS[1], "Off", "", ISS_OFF);
    IUFillSwitchVector(&GPSPowerSP, GPSPowerS, 2, getDeviceName(), "GPS Power", "", GPS_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillSwitch(&GPSStatusS[0], "Sleep", "", ISS_OFF);
    IUFillSwitch(&GPSStatusS[1], "Wake Up", "", ISS_OFF);
    IUFillSwitch(&GPSStatusS[2], "Restart", "", ISS_OFF);
    IUFillSwitchVector(&GPSStatusSP, GPSStatusS, 3, getDeviceName(), "GPS Status", "", GPS_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillSwitch(&GPSUpdateS[0], "Update GPS", "", ISS_OFF);
    IUFillSwitch(&GPSUpdateS[1], "Update Client", "", ISS_OFF);
    IUFillSwitchVector(&GPSUpdateSP, GPSUpdateS, 2, getDeviceName(), "GPS System", "", GPS_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillSwitch(&AltDecPecS[0], "Enable", "", ISS_OFF);
    IUFillSwitch(&AltDecPecS[1], "Disable", "", ISS_OFF);
    IUFillSwitchVector(&AltDecPecSP, AltDecPecS, 2, getDeviceName(), "Alt/Dec PEC", "", GPS_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillSwitch(&AzRaPecS[0], "Enable", "", ISS_OFF);
    IUFillSwitch(&AzRaPecS[1], "Disable", "", ISS_OFF);
    IUFillSwitchVector(&AzRaPecSP, AzRaPecS, 2, getDeviceName(), "Az/RA PEC", "", GPS_TAB, IP_RW, ISR_1OFMANY, 0,
                       IPS_IDLE);

    IUFillSwitch(&SelenSyncS[0], "Sync", "", ISS_OFF);
    IUFillSwitchVector(&SelenSyncSP, SelenSyncS, 1, getDeviceName(), "Selenographic Sync", "", GPS_TAB, IP_RW,
                       ISR_ATMOST1, 0, IPS_IDLE);

    IUFillSwitch(&AltDecBacklashS[0], "Activate", "", ISS_OFF);
    IUFillSwitchVector(&AltDecBacklashSP, AltDecBacklashS, 1, getDeviceName(), "Alt/Dec Anti-backlash", "", GPS_TAB,
                       IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    IUFillSwitch(&AzRaBacklashS[0], "Activate", "", ISS_OFF);
    IUFillSwitchVector(&AzRaBacklashSP, AzRaBacklashS, 1, getDeviceName(), "Az/Ra Anti-backlash", "", GPS_TAB, IP_RW,
                       ISR_ATMOST1, 0, IPS_IDLE);

    IUFillSwitch(&OTAUpdateS[0], "Update", "", ISS_OFF);
    IUFillSwitchVector(&OTAUpdateSP, OTAUpdateS, 1, getDeviceName(), "OTA Update", "", GPS_TAB, IP_RW, ISR_ATMOST1, 0,
                       IPS_IDLE);

    IUFillNumber(&OTATempN[0], "Temp", "", "%03g", -200.0, 500.0, 0.0, 0);
    IUFillNumberVector(&OTATempNP, OTATempN, 1, getDeviceName(), "OTA Temp (C)", "", GPS_TAB, IP_RO, 0, IPS_IDLE);

    GuideRateNP[0].fill("GUIDE_RATE_WE", "W/E Rate", "%g",  0, 2, 0.05, 0.5);
    GuideRateNP[1].fill("GUIDE_RATE_NS", "N/S Rate", "%g", 0, 2, 0.05, 0.5);
    GuideRateNP.fill(getDeviceName(), "GUIDE_RATE", "Custom Guide Rate", MOTION_TAB, IP_RW, 0, IPS_IDLE);

    // Custom track rates are signed axis rates: RA/Az axis and Dec/Alt axis
    TrackRateNP[AXIS_RA].fill("TRACK_RATE_RA", "RA/Az axis (arcsecs/s)", "%.6f", -LX200GPS_MAX_CUSTOM_RATE,
                              LX200GPS_MAX_CUSTOM_RATE, 0.000001, 0);
    TrackRateNP[AXIS_DE].fill("TRACK_RATE_DE", "DE/Alt axis (arcsecs/s)", "%.6f", -LX200GPS_MAX_CUSTOM_RATE,
                              LX200GPS_MAX_CUSTOM_RATE, 0.000001, 0);

    CurrentAltAzNP[0].fill("ALT", "Alt D:M:S", "%10.6m", -90., 90., 0., 0.);
    CurrentAltAzNP[1].fill("AZ", "Az D:M:S", "%10.6m", 0., 360., 0., 0.);
    CurrentAltAzNP.fill(getDeviceName(), "HORIZONTAL_COORD", "Horizontal Coords", MAIN_CONTROL_TAB, IP_RO, 0,
                        IPS_IDLE);

    MountAxesNP[0].fill("PRIMARY", "Primary (Az or HA, deg)", "%.4f", -360., 360., 0., 0.);
    MountAxesNP[1].fill("SECONDARY", "Secondary (Alt or Dec, deg)", "%.4f", -90., 90., 0., 0.);
    MountAxesNP.fill(getDeviceName(), "MOUNT_AXES", "Mount Axes", MOTION_TAB, IP_RO, 0, IPS_IDLE);

    MountTypeSP.reset();
    MountTypeSP[MOUNT_ALTAZ].setState(ISS_ON);

    return true;
}

void LX200GPS::ISGetProperties(const char *dev)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) != 0)
        return;

    // process parent first
    LX200Autostar::ISGetProperties(dev);

    /*
    if (isConnected())
    {
        defineProperty(&GPSPowerSP);
        defineProperty(&GPSStatusSP);
        defineProperty(&GPSUpdateSP);
        defineProperty(&AltDecPecSP);
        defineProperty(&AzRaPecSP);
        defineProperty(&SelenSyncSP);
        defineProperty(&AltDecBacklashSP);
        defineProperty(&AzRaBacklashSP);
        defineProperty(&OTATempNP);
        defineProperty(&OTAUpdateSP);
    }
    */
}

bool LX200GPS::updateProperties()
{
    LX200Autostar::updateProperties();

    if (isConnected())
    {
        defineProperty(&GPSPowerSP);
        defineProperty(&GPSStatusSP);
        defineProperty(&GPSUpdateSP);
        defineProperty(&AltDecPecSP);
        defineProperty(&AzRaPecSP);
        defineProperty(&SelenSyncSP);
        defineProperty(&AltDecBacklashSP);
        defineProperty(&AzRaBacklashSP);
        defineProperty(&OTATempNP);
        defineProperty(&OTAUpdateSP);
        defineProperty(GuideRateNP);
    }
    else
    {
        deleteProperty(GPSPowerSP.name);
        deleteProperty(GPSStatusSP.name);
        deleteProperty(GPSUpdateSP.name);
        deleteProperty(AltDecPecSP.name);
        deleteProperty(AzRaPecSP.name);
        deleteProperty(SelenSyncSP.name);
        deleteProperty(AltDecBacklashSP.name);
        deleteProperty(AzRaBacklashSP.name);
        deleteProperty(OTATempNP.name);
        deleteProperty(OTAUpdateSP.name);
        deleteProperty(GuideRateNP);
        deleteProperty(CurrentAltAzNP);
        deleteProperty(MountAxesNP);

        canGetAltAz          = false;
        canGetSiderealTime   = false;
        customTrackingActive = false;
        mountTrackingOff     = false;
        customDirection[AXIS_RA] = customDirection[AXIS_DE] = 0;
    }

    return true;
}

bool LX200GPS::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    int index = 0;
    char msg[64];

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        /* GPS Power */
        if (!strcmp(name, GPSPowerSP.name))
        {
            int ret = 0;

            if (IUUpdateSwitch(&GPSPowerSP, states, names, n) < 0)
                return false;

            index = IUFindOnSwitchIndex(&GPSPowerSP);
            if (index == 0)
                ret = turnGPSOn(PortFD);
            else
                ret = turnGPSOff(PortFD);

            INDI_UNUSED(ret);
            GPSPowerSP.s = IPS_OK;
            IDSetSwitch(&GPSPowerSP, index == 0 ? "GPS System is ON" : "GPS System is OFF");
            return true;
        }

        /* GPS Status Update */
        if (!strcmp(name, GPSStatusSP.name))
        {
            int ret = 0;

            if (IUUpdateSwitch(&GPSStatusSP, states, names, n) < 0)
                return false;

            index = IUFindOnSwitchIndex(&GPSStatusSP);

            if (index == 0)
            {
                ret = gpsSleep(PortFD);
                strncpy(msg, "GPS system is in sleep mode.", 64);
            }
            else if (index == 1)
            {
                ret = gpsWakeUp(PortFD)
                      strncpy(msg, "GPS system is reactivated.", 64);
            }
            else
            {
                ret = gpsRestart(PortFD)
                      strncpy(msg, "GPS system is restarting...", 64);
                sendScopeTime();
                sendScopeLocation();
            }

            INDI_UNUSED(ret);
            GPSStatusSP.s = IPS_OK;
            IDSetSwitch(&GPSStatusSP, "%s", msg);
            return true;
        }

        /* GPS Update */
        if (!strcmp(name, GPSUpdateSP.name))
        {
            if (IUUpdateSwitch(&GPSUpdateSP, states, names, n) < 0)
                return false;

            index = IUFindOnSwitchIndex(&GPSUpdateSP);

            GPSUpdateSP.s = IPS_OK;

            if (index == 0)
            {
                IDSetSwitch(&GPSUpdateSP, "Updating GPS system. This operation might take few minutes to complete...");
                if (updateGPS_System(PortFD))
                {
                    IDSetSwitch(&GPSUpdateSP, "GPS system update successful.");
                    sendScopeTime();
                    sendScopeLocation();
                }
                else
                {
                    GPSUpdateSP.s = IPS_IDLE;
                    IDSetSwitch(&GPSUpdateSP, "GPS system update failed.");
                }
            }
            else
            {
                sendScopeTime();
                sendScopeLocation();
                IDSetSwitch(&GPSUpdateSP, "Client time and location is synced to LX200 GPS Data.");
            }
            return true;
        }

        /* Alt Dec Periodic Error correction */
        if (!strcmp(name, AltDecPecSP.name))
        {
            int ret = 0;

            if (IUUpdateSwitch(&AltDecPecSP, states, names, n) < 0)
                return false;

            index = IUFindOnSwitchIndex(&AltDecPecSP);

            if (index == 0)
            {
                ret = enableDecAltPec(PortFD);
                strncpy(msg, "Alt/Dec Compensation Enabled.", 64);
            }
            else
            {
                ret = disableDecAltPec(PortFD);
                strncpy(msg, "Alt/Dec Compensation Disabled.", 64);
            }

            INDI_UNUSED(ret);
            AltDecPecSP.s = IPS_OK;
            IDSetSwitch(&AltDecPecSP, "%s", msg);

            return true;
        }

        /* Az RA periodic error correction */
        if (!strcmp(name, AzRaPecSP.name))
        {
            int ret = 0;

            if (IUUpdateSwitch(&AzRaPecSP, states, names, n) < 0)
                return false;

            index = IUFindOnSwitchIndex(&AzRaPecSP);

            if (index == 0)
            {
                ret = enableRaAzPec(PortFD);
                strncpy(msg, "Ra/Az Compensation Enabled.", 64);
            }
            else
            {
                ret = disableRaAzPec(PortFD);
                strncpy(msg, "Ra/Az Compensation Disabled.", 64);
            }

            INDI_UNUSED(ret);
            AzRaPecSP.s = IPS_OK;
            IDSetSwitch(&AzRaPecSP, "%s", msg);

            return true;
        }

        if (!strcmp(name, AltDecBacklashSP.name))
        {
            int ret = 0;

            ret = activateAltDecAntiBackSlash(PortFD);
            INDI_UNUSED(ret);
            AltDecBacklashSP.s = IPS_OK;
            IDSetSwitch(&AltDecBacklashSP, "Alt/Dec Anti-backlash enabled");
            return true;
        }

        if (!strcmp(name, AzRaBacklashSP.name))
        {
            int ret = 0;

            ret = activateAzRaAntiBackSlash(PortFD);
            AzRaBacklashSP.s = IPS_OK;
            INDI_UNUSED(ret);
            IDSetSwitch(&AzRaBacklashSP, "Az/Ra Anti-backlash enabled");
            return true;
        }

        if (!strcmp(name, OTAUpdateSP.name))
        {
            IUResetSwitch(&OTAUpdateSP);

            if (getOTATemp(PortFD, &OTATempNP.np[0].value) < 0)
            {
                OTAUpdateSP.s = IPS_ALERT;
                OTATempNP.s   = IPS_ALERT;
                IDSetNumber(&OTATempNP, "Error: OTA temperature read timed out.");
                return false;
            }
            else
            {
                OTAUpdateSP.s = IPS_OK;
                OTATempNP.s   = IPS_OK;
                IDSetNumber(&OTATempNP, nullptr);
                IDSetSwitch(&OTAUpdateSP, nullptr);
                return true;
            }
        }
    }

    return LX200Autostar::ISNewSwitch(dev, name, states, names, n);
}

bool LX200GPS::updateTime(ln_date *utc, double utc_offset)
{
    ln_zonedate ltm;

    if (isSimulation())
        return true;

    JD = ln_get_julian_day(utc);

    LOGF_DEBUG("New JD is %.2f", JD);

    ln_date_to_zonedate(utc, &ltm, utc_offset * 3600);

    LOGF_DEBUG("Local time is %02d:%02d:%02g", ltm.hours, ltm.minutes, ltm.seconds);

    // Set Local Time
    if (setLocalTime24(ltm.hours, ltm.minutes, ltm.seconds) == false)
    {
        LOG_ERROR("Error setting local time time.");
        return false;
    }

    // UTC Date, it's not Local for LX200GPS
    if (setLocalDate(utc->days, utc->months, utc->years) == false)
    {
        LOG_ERROR("Error setting UTC date.");
        return false;
    }

    // Meade defines UTC Offset as the offset ADDED to local time to yield UTC, which
    // is the opposite of the standard definition of UTC offset!
    if (setUTCOffset(utc_offset) == false)
    {
        LOG_ERROR("Error setting UTC Offset.");
        return false;
    }

    LOG_INFO("Time updated, updating planetary data...");
    return true;
}

bool LX200GPS::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{

    if (strcmp(name, "GUIDE_RATE") == 0)
    {
        GuideRateNP[0].value = values[0];
        GuideRateNP[1].value = values[1];
        GuideRateNP.apply();
        LOG_INFO("Guide rate set in INDI driver, please check mount is set to the same values");
    }

    return LX200Autostar::ISNewNumber(dev, name, values, names, n);
}

bool LX200GPS::saveConfigItems(FILE *fp)
{
    LX200Autostar::saveConfigItems(fp);
    GuideRateNP.save(fp);
    return true;
}

bool LX200GPS::UnPark()
{
    int ret = 0;

    ret = initTelescope(PortFD);

    INDI_UNUSED(ret);
    TrackState = SCOPE_IDLE;
    SetParked(false);
    return true;
}

bool LX200GPS::Disconnect()
{
    // don't leave the axes moving; closing the port discards output that has not been sent yet
    if (customTrackingActive && stopCustomTracking() && !isSimulation())
        tcdrain(PortFD);

    return LX200Autostar::Disconnect();
}

void LX200GPS::getBasicData()
{
    LX200Autostar::getBasicData();

    if (!HasTrackRate())
        return;

    // The mount axes are Az/Alt from :GA# and :GZ#, or on a polar mount the hour angle from :GS# and RA/Dec.
    // Not every LX200 controller answers these, so only the ones the alignment needs are tried.
    bool polar = IUFindOnSwitchIndex(&AlignmentSP) == LX200_ALIGN_POLAR;
    double alt = 0, az = 0, lst = 0;
    canGetAltAz = isSimulation() || (!polar && getLX200Alt(PortFD, &alt) == 0 && getLX200Az(PortFD, &az) == 0);
    canGetSiderealTime = isSimulation() || (polar && getSDTime(PortFD, &lst) == 0);

    if (canGetAltAz)
        defineProperty(CurrentAltAzNP);

    if (polar ? canGetSiderealTime : canGetAltAz)
        defineProperty(MountAxesNP);
    else
        LOGF_WARN("The mount does not report %s, MOUNT_AXES is not available.",
                  polar ? "the sidereal time (:GS#)" : "Alt/Az (:GA#, :GZ#)");
}

void LX200GPS::updateMountAxes()
{
    double primary = 0, secondary = 0;
    bool ok = false;

    if (IUFindOnSwitchIndex(&AlignmentSP) == LX200_ALIGN_POLAR)
    {
        // hour angle and declination, the RA/Dec just read by the status update
        double lst = 0;
        if (isSimulation())
        {
            lst = get_local_sidereal_time(LocationNP[LOCATION_LONGITUDE].getValue());
            ok  = true;
        }
        else
            ok = canGetSiderealTime && getSDTime(PortFD, &lst) == 0;
        primary   = rangeHA(lst - currentRA) * 15.0;
        secondary = currentDEC;
    }
    else if (canGetAltAz && CurrentAltAzNP.getState() == IPS_OK)
    {
        primary   = CurrentAltAzNP[1].getValue();
        secondary = CurrentAltAzNP[0].getValue();
        ok        = true;
    }

    if (ok)
    {
        MountAxesNP[0].setValue(primary);
        MountAxesNP[1].setValue(secondary);
        MountAxesNP.setState(IPS_OK);
    }
    else
        MountAxesNP.setState(IPS_ALERT);
    MountAxesNP.apply();
}

bool LX200GPS::ReadScopeStatus()
{
    TelescopeStatus previousState = TrackState;

    if (!LX200Autostar::ReadScopeStatus())
        return false;

    // custom tracking resumes when a slew completes
    if (HasTrackRate() && previousState == SCOPE_SLEWING && TrackState == SCOPE_TRACKING &&
            TrackModeSP.findOnSwitchIndex() == TRACK_CUSTOM && !startCustomTracking())
        LOG_ERROR("Failed to resume custom tracking.");

    if (canGetAltAz)
    {
        double alt = simAlt, az = simAz;
        if (isSimulation() || (getLX200Alt(PortFD, &alt) == 0 && getLX200Az(PortFD, &az) == 0))
        {
            CurrentAltAzNP[0].setValue(alt);
            CurrentAltAzNP[1].setValue(az);
            CurrentAltAzNP.setState(IPS_OK);
        }
        else
            CurrentAltAzNP.setState(IPS_ALERT);
        CurrentAltAzNP.apply();
    }

    if (canGetAltAz || canGetSiderealTime)
        updateMountAxes();

    return true;
}

bool LX200GPS::Goto(double ra, double dec)
{
    // the mount slews and tracks by itself, custom tracking resumes when the slew completes
    if (customTrackingActive && !stopCustomTracking())
        return false;
    if (mountTrackingOff && !setMountTracking(true))
        return false;

    return LX200Autostar::Goto(ra, dec);
}

bool LX200GPS::Park()
{
    if (customTrackingActive && !stopCustomTracking())
        return false;
    if (mountTrackingOff && !setMountTracking(true))
        return false;

    return LX200Autostar::Park();
}

bool LX200GPS::Abort()
{
    if (customTrackingActive && stopCustomTracking())
        TrackState = SCOPE_IDLE;

    return LX200Autostar::Abort();
}

// the INDI overload, called while tracking
bool LX200GPS::SetTrackMode(uint8_t mode)
{
    if (!HasTrackRate())
        return LX200Autostar::SetTrackMode(mode);

    if (mode == TRACK_CUSTOM)
        return startCustomTracking();

    // leaving custom tracking, give tracking back to the mount
    if (customTrackingActive && !(stopCustomTracking() && setMountTracking(true)))
        return false;

    return LX200Autostar::SetTrackMode(mode);
}

bool LX200GPS::SetTrackEnabled(bool enabled)
{
    if (enabled && HasTrackRate() && TrackModeSP.findOnSwitchIndex() == TRACK_CUSTOM)
        return startCustomTracking();

    if (customTrackingActive && !stopCustomTracking())
        return false;

    if (!enabled)
        return setMountTracking(false);

    if (IUFindOnSwitchIndex(&AlignmentSP) == LX200_ALIGN_LAND)
    {
        LOG_WARN("The mount does not track in Land alignment. Select the Polar or AltAz alignment.");
        return false;
    }

    return setMountTracking(true) && LX200Autostar::SetTrackMode(TrackModeSP.findOnSwitchIndex());
}

// only called by INDI::Telescope while tracking in custom mode
bool LX200GPS::SetTrackRate(double raRate, double deRate)
{
    // TrackRateNP already holds the new rates, so starting picks them up
    if (!customTrackingActive)
        return startCustomTracking();

    return moveCustomAxis(AXIS_RA, raRate) && moveCustomAxis(AXIS_DE, deRate);
}

bool LX200GPS::startCustomTracking()
{
    // the mount's own tracking would add to the axis rates
    if (!setMountTracking(false))
        return false;

    if (!customTrackingActive)
        simAxesFromRaDec();

    if (!moveCustomAxis(AXIS_RA, TrackRateNP[AXIS_RA].getValue()) ||
            !moveCustomAxis(AXIS_DE, TrackRateNP[AXIS_DE].getValue()))
    {
        stopCustomTracking();
        return false;
    }

    customTrackingActive = true;
    LOGF_DEBUG("Custom tracking at axis rates %.4f, %.4f arcsec/s.", TrackRateNP[AXIS_RA].getValue(),
               TrackRateNP[AXIS_DE].getValue());
    return true;
}

bool LX200GPS::stopCustomTracking()
{
    if (!isSimulation() && HaltMovement(PortFD, LX200_ALL) < 0)
    {
        LOG_ERROR("Failed to stop custom tracking.");
        return false;
    }

    customDirection[AXIS_RA] = customDirection[AXIS_DE] = 0;
    customTrackingActive = false;
    LOG_DEBUG("Custom tracking stopped.");
    return true;
}

// Move one axis at a signed rate in arcsec/s. Positive rates move west on the RA/Az axis (increasing azimuth on
// an Alt-Az mount) and north on the Dec/Alt axis (increasing altitude).
bool LX200GPS::moveCustomAxis(INDI_EQ_AXIS axis, double rate)
{
    int direction = (rate > 0) - (rate < 0);
    int positive  = (axis == AXIS_RA) ? LX200_WEST : LX200_NORTH;
    int negative  = (axis == AXIS_RA) ? LX200_EAST : LX200_SOUTH;

    if (isSimulation())
    {
        customDirection[axis] = direction;
        return true;
    }

    // stop the move in the other direction
    if (customDirection[axis] != 0 && customDirection[axis] != direction)
    {
        if (HaltMovement(PortFD, customDirection[axis] > 0 ? positive : negative) < 0)
        {
            LOGF_ERROR("Failed to stop the %s axis.", axis == AXIS_RA ? "RA/Az" : "DE/Alt");
            return false;
        }
        customDirection[axis] = 0;
    }

    if (direction == 0)
        return true;

    // the move is sent with every rate, in case the mount applies a new rate only to a new move
    double degrees = std::fabs(rate) / 3600.0;
    int rc = (axis == AXIS_RA) ? setRAAzSlewRate(PortFD, degrees) : setDecAltSlewRate(PortFD, degrees);
    if (rc < 0 || MoveTo(PortFD, direction > 0 ? positive : negative) < 0)
    {
        LOGF_ERROR("Failed to move the %s axis.", axis == AXIS_RA ? "RA/Az" : "DE/Alt");
        return false;
    }

    customDirection[axis] = direction;
    return true;
}

// Switch the mount's own tracking off with the Land alignment, or back on with the alignment read from the mount
bool LX200GPS::setMountTracking(bool enabled)
{
    if (mountTrackingOff == !enabled)
        return true;

    int alignment = enabled ? IUFindOnSwitchIndex(&AlignmentSP) : LX200_ALIGN_LAND;
    if (!isSimulation() && setAlignmentMode(PortFD, alignment) < 0)
    {
        LOGF_ERROR("Failed to switch the mount's tracking %s.", enabled ? "on" : "off");
        return false;
    }

    mountTrackingOff = !enabled;
    return true;
}

void LX200GPS::mountSim()
{
    if (!customTrackingActive || TrackState != SCOPE_TRACKING)
    {
        LX200Autostar::mountSim();
        simAxesFromRaDec();
        return;
    }

    struct timeval now;
    gettimeofday(&now, nullptr);
    double dt = (simLastStep.tv_sec == 0) ? 0 : now.tv_sec - simLastStep.tv_sec + (now.tv_usec - simLastStep.tv_usec) / 1e6;
    simLastStep = now;

    double primary   = TrackRateNP[AXIS_RA].getValue() * dt / 3600.0;
    double secondary = TrackRateNP[AXIS_DE].getValue() * dt / 3600.0;

    if (IUFindOnSwitchIndex(&AlignmentSP) == LX200_ALIGN_POLAR)
    {
        // the sky moves west at the sidereal rate, the RA axis moves west at the primary rate
        currentRA  = range24(currentRA + (TRACKRATE_SIDEREAL * dt / 3600.0 - primary) / 15.0);
        currentDEC = rangeDec(currentDEC + secondary);
        simAxesFromRaDec();
    }
    else
    {
        simAz  = range360(simAz + primary);
        simAlt = std::max(-90.0, std::min(90.0, simAlt + secondary));

        INDI::IHorizontalCoordinates horizontal {simAz, simAlt};
        INDI::IEquatorialCoordinates equatorial {0, 0};
        INDI::IGeographicCoordinates observer = simObserver();
        INDI::HorizontalToEquatorial(&horizontal, &observer, ln_get_julian_from_sys(), &equatorial);
        currentRA  = equatorial.rightascension;
        currentDEC = equatorial.declination;
    }

    NewRaDec(currentRA, currentDEC);
}

INDI::IGeographicCoordinates LX200GPS::simObserver()
{
    return INDI::IGeographicCoordinates
    {
        range360(LocationNP[LOCATION_LONGITUDE].getValue()),
        LocationNP[LOCATION_LATITUDE].getValue(),
        LocationNP[LOCATION_ELEVATION].getValue()
    };
}

// keep the simulated Alt/Az in step with the simulated RA/Dec
void LX200GPS::simAxesFromRaDec()
{
    if (!isSimulation())
        return;

    INDI::IEquatorialCoordinates equatorial {currentRA, currentDEC};
    INDI::IHorizontalCoordinates horizontal {0, 0};
    INDI::IGeographicCoordinates observer = simObserver();
    INDI::EquatorialToHorizontal(&equatorial, &observer, ln_get_julian_from_sys(), &horizontal);
    simAz  = horizontal.azimuth;
    simAlt = horizontal.altitude;
}
