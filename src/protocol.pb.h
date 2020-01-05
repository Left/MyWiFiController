/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.4 at Sun Jan  5 21:36:23 2020. */

#ifndef PB_PROTOCOL_PB_H_INCLUDED
#define PB_PROTOCOL_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _MsgBack_ShowType {
    MsgBack_ShowType_SHOW = 0,
    MsgBack_ShowType_TUNE = 1,
    MsgBack_ShowType_ADDITIONAL = 2
} MsgBack_ShowType;
#define _MsgBack_ShowType_MIN MsgBack_ShowType_SHOW
#define _MsgBack_ShowType_MAX MsgBack_ShowType_ADDITIONAL
#define _MsgBack_ShowType_ARRAYSIZE ((MsgBack_ShowType)(MsgBack_ShowType_ADDITIONAL+1))

/* Struct definitions */
typedef struct _ParsedRemote {
    pb_callback_t remote;
    pb_callback_t key;
/* @@protoc_insertion_point(struct:ParsedRemote) */
} ParsedRemote;

typedef struct _Hello {
    int32_t versionMajor;
    int32_t versionMinor;
    int32_t versionLast;
    bool screenEnabled;
    pb_callback_t settings;
/* @@protoc_insertion_point(struct:Hello) */
} Hello;

typedef struct _MsgBack {
    int32_t id;
    pb_callback_t testMsg;
    bool has_unixtime;
    uint32_t unixtime;
    pb_callback_t textToShow;
    bool has_timeMsToShow;
    uint32_t timeMsToShow;
    bool has_showType;
    MsgBack_ShowType showType;
    bool has_relaysToSwitch;
    uint32_t relaysToSwitch;
    bool has_relaysToSwitchState;
    bool relaysToSwitchState;
    bool has_atxEnable;
    bool atxEnable;
    bool has_playMp3;
    uint32_t playMp3;
    bool has_volume;
    uint32_t volume;
    bool has_brightness;
    uint32_t brightness;
    bool has_pwmPin;
    uint32_t pwmPin;
    bool has_pwmValue;
    uint32_t pwmValue;
    bool has_pwmPeriod;
    uint32_t pwmPeriod;
    pb_callback_t ledValue;
    bool has_ledPeriod;
    uint32_t ledPeriod;
    pb_callback_t ledBaseColor;
    pb_callback_t ledBlinkColors;
    bool has_introduceYourself;
    bool introduceYourself;
    bool has_reboot;
    bool reboot;
    bool has_screenEnable;
    bool screenEnable;
/* @@protoc_insertion_point(struct:MsgBack) */
} MsgBack;

typedef struct _Relay {
    int32_t id;
    bool state;
/* @@protoc_insertion_point(struct:Relay) */
} Relay;

typedef struct _Msg {
    int32_t id;
    int32_t timeseq;
    bool has_hello;
    Hello hello;
    pb_callback_t irKeyPeriods;
    pb_callback_t debugLogMessage;
    pb_size_t relayStates_count;
    Relay relayStates[8];
    bool has_parsedRemote;
    ParsedRemote parsedRemote;
    bool has_buttonPressed;
    bool buttonPressed;
    bool has_weight;
    uint32_t weight;
    bool has_temp;
    float temp;
    bool has_humidity;
    float humidity;
    bool has_pressure;
    float pressure;
    bool has_potentiometer;
    uint32_t potentiometer;
    bool has_atxState;
    bool atxState;
/* @@protoc_insertion_point(struct:Msg) */
} Msg;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Hello_init_default                       {0, 0, 0, 0, {{NULL}, NULL}}
#define Relay_init_default                       {0, 0}
#define ParsedRemote_init_default                {{{NULL}, NULL}, {{NULL}, NULL}}
#define Msg_init_default                         {0, 0, false, Hello_init_default, {{NULL}, NULL}, {{NULL}, NULL}, 0, {Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default}, false, ParsedRemote_init_default, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define MsgBack_init_default                     {0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, false, 0, false, _MsgBack_ShowType_MIN, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, {{NULL}, NULL}, false, 0, false, 0, false, 0}
#define Hello_init_zero                          {0, 0, 0, 0, {{NULL}, NULL}}
#define Relay_init_zero                          {0, 0}
#define ParsedRemote_init_zero                   {{{NULL}, NULL}, {{NULL}, NULL}}
#define Msg_init_zero                            {0, 0, false, Hello_init_zero, {{NULL}, NULL}, {{NULL}, NULL}, 0, {Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero}, false, ParsedRemote_init_zero, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define MsgBack_init_zero                        {0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, false, 0, false, _MsgBack_ShowType_MIN, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, {{NULL}, NULL}, false, 0, false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define ParsedRemote_remote_tag                  1
#define ParsedRemote_key_tag                     2
#define Hello_versionMajor_tag                   1
#define Hello_versionMinor_tag                   2
#define Hello_versionLast_tag                    3
#define Hello_screenEnabled_tag                  4
#define Hello_settings_tag                       5
#define MsgBack_id_tag                           1
#define MsgBack_introduceYourself_tag            200
#define MsgBack_reboot_tag                       201
#define MsgBack_screenEnable_tag                 202
#define MsgBack_testMsg_tag                      3
#define MsgBack_unixtime_tag                     4
#define MsgBack_textToShow_tag                   5
#define MsgBack_timeMsToShow_tag                 6
#define MsgBack_showType_tag                     7
#define MsgBack_relaysToSwitch_tag               8
#define MsgBack_relaysToSwitchState_tag          9
#define MsgBack_atxEnable_tag                    10
#define MsgBack_playMp3_tag                      11
#define MsgBack_volume_tag                       12
#define MsgBack_brightness_tag                   13
#define MsgBack_pwmPin_tag                       14
#define MsgBack_pwmValue_tag                     15
#define MsgBack_pwmPeriod_tag                    16
#define MsgBack_ledValue_tag                     17
#define MsgBack_ledPeriod_tag                    18
#define MsgBack_ledBaseColor_tag                 19
#define MsgBack_ledBlinkColors_tag               20
#define Relay_id_tag                             1
#define Relay_state_tag                          2
#define Msg_id_tag                               1
#define Msg_timeseq_tag                          2
#define Msg_hello_tag                            3
#define Msg_irKeyPeriods_tag                     4
#define Msg_debugLogMessage_tag                  5
#define Msg_relayStates_tag                      6
#define Msg_parsedRemote_tag                     7
#define Msg_buttonPressed_tag                    8
#define Msg_weight_tag                           9
#define Msg_temp_tag                             10
#define Msg_humidity_tag                         11
#define Msg_pressure_tag                         12
#define Msg_potentiometer_tag                    13
#define Msg_atxState_tag                         14

/* Struct field encoding specification for nanopb */
extern const pb_field_t Hello_fields[6];
extern const pb_field_t Relay_fields[3];
extern const pb_field_t ParsedRemote_fields[3];
extern const pb_field_t Msg_fields[15];
extern const pb_field_t MsgBack_fields[23];

/* Maximum encoded size of messages (where known) */
/* Hello_size depends on runtime parameters */
#define Relay_size                               13
/* ParsedRemote_size depends on runtime parameters */
/* Msg_size depends on runtime parameters */
/* MsgBack_size depends on runtime parameters */

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define PROTOCOL_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif