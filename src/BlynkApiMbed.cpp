/**
 * @file       BlynkApiArduino.h
 * @author     Volodymyr Shymanskyy
 * @license    This project is released under the MIT License (MIT)
 * @copyright  Copyright (c) 2015 Volodymyr Shymanskyy
 * @date       Mar 2015
 * @brief
 *
 */

#ifndef BlynkApiArduino_h
#define BlynkApiArduino_h

#include "mbed.h"
#include <Blynk/BlynkApi.h>
#include <iostream>
#include <vector>
#include <string>
using namespace std;

static Timer  blynk_millis_timer;
static Ticker blynk_waker;

template<class Proto>
void BlynkApi<Proto>::Init()
{
    try{
    blynk_waker.attach(&blynk_wake, 2.0);
    }catch(...){
    cout << "Exceptions";
    }
    try{
    blynk_millis_timer.start();
    }catch(...){
    cout << "Exceptions";
    }
    
}

template<class Proto>
BLYNK_FORCE_INLINE
millis_time_t BlynkApi<Proto>::getMillis()
{
    return blynk_millis_timer.read_ms();
}

#ifdef BLYNK_NO_INFO

template<class Proto>
BLYNK_FORCE_INLINE
void BlynkApi<Proto>::sendInfo() {


}

#else

template<class Proto>
BLYNK_FORCE_INLINE
void BlynkApi<Proto>::sendInfo()
{
        static const vector<profile> BLYNK_PROGMEM[] =
        BLYNK_PARAM_KV("ver"    , BLYNK_VERSION)
        BLYNK_PARAM_KV("h-beat" , BLYNK_TOSTRING(BLYNK_HEARTBEAT))
        BLYNK_PARAM_KV("buff-in", BLYNK_TOSTRING(BLYNK_MAX_READBYTES))
#ifdef BLYNK_INFO_DEVICE
        BLYNK_PARAM_KV("dev"    , BLYNK_INFO_DEVICE)
#endif
#ifdef BLYNK_INFO_CPU
        BLYNK_PARAM_KV("cpu"    , BLYNK_INFO_CPU)
#endif
#ifdef BLYNK_INFO_CONNECTION
        BLYNK_PARAM_KV("con"    , BLYNK_INFO_CONNECTION)
#endif
        BLYNK_PARAM_KV("build"  , __DATE__ " " __TIME__)
    ;
    const size_t profile_len = sizeof(profile)-1;

#ifdef BLYNK_HAS_PROGMEM
    vector<char> mem[profile_len];
    memcpy_P(mem, profile, profile_len);
    static_cast<Proto*>(this)->sendCmd(BLYNK_CMD_INTERNAL, 0, mem, profile_len);
#else
    static_cast<Proto*>(this)->sendCmd(BLYNK_CMD_INTERNAL, 0, profile, profile_len);
#endif
    return;
}

#endif

template<class Proto>
BLYNK_FORCE_INLINE
void BlynkApi<Proto>::processCmd(const void* buff, size_t len)
{
    BlynkParam paramobj((void*)buff, len);
    BlynkParam::iterator it = paramobj.begin();
    if (it >= paramobj.end())
        return;
    const char* Paramcmd = it.asStr();
    if(!Paramcmd)
        return;
    uint16_t cmd16;
    memcpy(&cmd16, cmd, sizeof(cmd16));
    if (++it >= paramobj.end())
        return;

#if defined(analogInputToDigitalPin)
    // Good! Analog pins can be referenced on this device by name.
    const uint8_t pin = (it.asStr()[0] == 'A') ?
                         analogInputToDigitalPin(atoi(it.asStr()+1)) :
                         it.asInt();
#else
    #if defined(BLYNK_DEBUG_ALL)
        #pragma message "analogInputToDigitalPin not defined"
    #endif
    const uint8_t pin = it.asInt();
#endif

    switch(cmd16) {

#ifndef BLYNK_NO_BUILTIN

    case BLYNK_HW_PM: {
        while (it < param.end()) {
            ++it;
            if (!strcmp(it.asStr(), "in")) {
                //pinMode(pin, INPUT);
            } else if (!strcmp(it.asStr(), "out") || !strcmp(it.asStr(), "pwm")) {
                //pinMode(pin, OUTPUT);
            } else {
#ifdef BLYNK_DEBUG
                BLYNK_LOG4(BLYNK_F("Invalid pin "), pin, BLYNK_F(" mode "), it.asStr());
#endif
            }
            ++it;
        }
    } break;
    case BLYNK_HW_DR: {
        DigitalIn DigitalObj((PinName)pin);
        char mem[16];
        BlynkParam rspobj(mem, 0, sizeof(mem));
        rspobj.add("dw");
        rspobj.add(pin);
        rspobj.add(int(DigitalObj));
        static_cast<Proto*>(this)->sendCmd(BLYNK_CMD_HARDWARE, 0, rsp.getBuffer(), rsp.getLength()-1);
    } break;
    case BLYNK_HW_DW: {
        // Should be 1 parameter (value)
        if (++it >= param.end())
            return;

        //BLYNK_LOG("digitalWrite %d -> %d", pin, it.asInt());
        DigitalOut DoutObj((PinName)pin);
        DoutObj = it.asInt() ? 1 : 0;
    } break;
    case BLYNK_HW_AR: {
        AnalogIn AnalogObj((PinName)pin);
        char mem[16];
        BlynkParam rspobj(mem, 0, sizeof(mem));
        rspobj.add("aw");
        rspobj.add(pin);
        rspobj.add(int(AnalogObj.read() * 1024));
        static_cast<Proto*>(this)->sendCmd(BLYNK_CMD_HARDWARE, 0, rsp.getBuffer(), rsp.getLength()-1);
    } break;
    case BLYNK_HW_AW: {
        // TODO: Not supported yet
    } break;

#endif

    case BLYNK_HW_VR: {
        BlynkReq req = { pin };
        WidgetReadHandler handler = GetReadHandler(pin);
        if (handler && (handler != BlynkWidgetRead)) {
            handler(req);
        } else {
            BlynkWidgetReadDefault(req);
        }
    } break;
    case BLYNK_HW_VW: {
        ++it;
        char* start = (char*)it.asStr();
        if(!start)
            break;
        BlynkParam param2(start, len - (start - (char*)buff));
        BlynkReq req = { pin };
        WidgetWriteHandler handler = GetWriteHandler(pin);
        if (handler && (handler != BlynkWidgetWrite)) {
            handler(req, param2);
        } else {
            BlynkWidgetWriteDefault(req, param2);
        }
    } break;
    default:
        BLYNK_LOG2(BLYNK_F("Invalid HW cmd: "), cmd);
        static_cast<Proto*>(this)->sendCmd(BLYNK_CMD_RESPONSE, static_cast<Proto*>(this)->currentMsgId, NULL, BLYNK_ILLEGAL_COMMAND);
    }
}

#endif
