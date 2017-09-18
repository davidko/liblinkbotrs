
// Copyright (c) 2013-2017 Barobo, Inc.
//
// This file is part of liblinkbot.
//
// liblinkbot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// liblinkbot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with liblinkbot.  If not, see <http://www.gnu.org/licenses/>.

#ifndef LINKBOT_LINKBOT_H
#define LINKBOT_LINKBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#define LINKBOT_MAX_SPEED 200

enum LinkbotButtonState {
    LINKBOT_BUTTON_STATE_UP,
    LINKBOT_BUTTON_STATE_DOWN
};

enum LinkbotButton {
    LINKBOT_BUTTON_POWER,
    LINKBOT_BUTTON_A,
    LINKBOT_BUTTON_B
};

enum LinkbotDirection {
    LINKBOT_BACKWARD=-1,
    LINKBOT_NEUTRAL=0,
    LINKBOT_FORWARD=1,
    LINKBOT_POSITIVE,
    LINKBOT_NEGATIVE
};

enum LinkbotFormFactor {
    LINKBOT_FORM_FACTOR_I,
    LINKBOT_FORM_FACTOR_L,
    LINKBOT_FORM_FACTOR_T
};

enum LinkbotJoint {
    LINKBOT_JOINT_ONE,
    LINKBOT_JOINT_TWO,
    LINKBOT_JOINT_THREE
};

// Keep in sync with enumeration in robot.proto
enum LinkbotJointState {
    LINKBOT_JOINT_STATE_COAST,
    LINKBOT_JOINT_STATE_HOLD,
    LINKBOT_JOINT_STATE_MOVING,
    LINKBOT_JOINT_STATE_FAILURE = 4,
    LINKBOT_JOINT_STATE_POWER = 5,
};

typedef void (*LinkbotButtonEventCallback)(LinkbotButton button, LinkbotButtonState event, int timestamp, void* userData);
// EncoderEventCallback's anglePosition parameter is reported in degrees.
typedef void (*LinkbotEncoderEventCallback)(int jointNo, double anglePosition, int timestamp, void* userData);
typedef void (*LinkbotJointEventCallback)(int jointNo, LinkbotJointState event, int timestamp, void* userData);
typedef void (*LinkbotAccelerometerEventCallback)(double x, double y, double z, int timestamp, void* userData);
typedef void (*LinkbotConnectionTerminatedCallback)(int timestamp, void* userData);

typedef struct Linkbot Linkbot;

//Linkbot* linkbotFromTcpEndpoint(const char* host, const char* service);
Linkbot* linkbotFromSerialId(const char* serialId);
void linkbotDelete(Linkbot* l);

/* MISC */
int linkbotWriteEeprom(Linkbot* l, unsigned int address, const char *data, unsigned int size);

/* GETTERS */
int linkbotGetAccelerometer(Linkbot* l, double *x, double *y, double *z);
int linkbotGetBatteryVoltage(Linkbot* l, double*);
int linkbotGetFormFactor(Linkbot* l, LinkbotFormFactor *form);
int linkbotGetJointAngles(Linkbot* l, int* timestamp, double *j1, double *j2,
                          double *j3);
int linkbotGetJointSpeeds(Linkbot* l, double *s1, double *s2, double *s3);
int linkbotGetJointStates(Linkbot*, int *timestamp, LinkbotJointState *j1,
                          LinkbotJointState *j2,
                          LinkbotJointState *j3);
int linkbotGetLedColor(Linkbot* l, int *r, int *g, int *b);

int linkbotGetVersionString (Linkbot* l, char* v, unsigned n);
// Copy the firmware version string into the character array pointed to by `v` and null-terminate
// the array. `n` must be the size of the character array. If `n` is larger than required, the
// excess space in the array will be null-padded. If `n` is smaller than required, the version
// string will be truncated to fit and null-terminated.

int linkbotGetSerialId(Linkbot* l, char* serialId);
int linkbotGetJointSafetyThresholds(Linkbot* l, int*, int*, int*);
int linkbotGetJointSafetyAngles(Linkbot* l, double*, double*, double*);

/* SETTERS */
int linkbotSetAlphaI(Linkbot* l, int mask,
    double a1, double a2, double a3);
int linkbotSetAlphaF(Linkbot* l, int mask,
    double a1, double a2, double a3);
int linkbotResetEncoderRevs(Linkbot* l);
int linkbotSetBuzzerFrequency(Linkbot* l, float freq);
int linkbotSetJointSpeeds(Linkbot* l, int mask, double j1, double j2,
                          double j3);
int linkbotSetJointStates(Linkbot* l, int mask,
        LinkbotJointState s1, double d1,
        LinkbotJointState s2, double d2,
        LinkbotJointState s3, double d3);
int linkbotSetJointStatesTimed(Linkbot* l, int mask,
        LinkbotJointState s1, double d1, double timeout1, LinkbotJointState end1,
        LinkbotJointState s2, double d2, double timeout2, LinkbotJointState end2,
        LinkbotJointState s3, double d3, double timeout3, LinkbotJointState end3);
int linkbotSetLedColor(Linkbot* l, int r, int g, int b);
int linkbotSetJointSafetyThresholds(Linkbot* l, int mask, int t1, int t2, int t3);
int linkbotSetJointSafetyAngles(Linkbot* l, int mask, double t1, double t2, double t3);
int linkbotSetPeripheralResetMask(Linkbot* l, int mask, int peripheralMask);

/* MOVEMENT */
int linkbotMoveAccel(Linkbot* l, int mask, int relativeMask,
    double a0, double timeout0, LinkbotJointState endstate0,
    double a1, double timeout1, LinkbotJointState endstate1,
    double a2, double timeout2, LinkbotJointState endstate2);
int linkbotMoveSmooth(Linkbot* l,
    int mask, int relativeMask, double a0, double a1, double a2);
int linkbotMoveContinuous(Linkbot* l, int mask,
                          double d1,
                          double d2,
                          double d3);
int linkbotMoveWait(Linkbot*, int mask);
int linkbotDrive(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotDriveTo(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotMotorPower(Linkbot*, int mask, int m1, int m2, int m3);
int linkbotMove(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotMoveTo(Linkbot*, int mask, double j1, double j2, double j3);
int linkbotStop(Linkbot*, int mask);

/* CALLBACKS */
#define LINKBOT_DECL_EVENT_CALLBACK(cbname, ...) \
    int linkbotSet##cbname(Linkbot* l, Linkbot##cbname cb, __VA_ARGS__)
LINKBOT_DECL_EVENT_CALLBACK(ButtonEventCallback, void* userData);
LINKBOT_DECL_EVENT_CALLBACK(EncoderEventCallback, float granularity, void* userData);
LINKBOT_DECL_EVENT_CALLBACK(JointEventCallback, void* userData);
LINKBOT_DECL_EVENT_CALLBACK(AccelerometerEventCallback, void* userData);
LINKBOT_DECL_EVENT_CALLBACK(ConnectionTerminatedCallback, void* userData);
#undef LINKBOT_DECL_EVENT_CALLBACK

#ifdef __cplusplus
} // extern "C"
#endif

#undef LIBLINKBOT_EXPORT

#endif
