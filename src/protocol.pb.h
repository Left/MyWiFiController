/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.2 */

#ifndef PB_PROTOCOL_PB_H_INCLUDED
#define PB_PROTOCOL_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
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

/* Struct definitions */
typedef struct _ParsedRemote {
    pb_callback_t remote;
    pb_callback_t key;
} ParsedRemote;

typedef struct _Hello {
    int32_t versionMajor;
    int32_t versionMinor;
    int32_t versionLast;
    bool screenEnabled;
    pb_callback_t settings;
} Hello;

typedef struct _Relay {
    int32_t id;
    bool state;
} Relay;

typedef struct _ScreenContent {
    uint32_t width;
    uint32_t height;
    pb_callback_t content;
} ScreenContent;

typedef struct _ScreenOffset {
    int32_t x;
    int32_t y;
    uint32_t atMs;
} ScreenOffset;

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
    bool has_buttonPressedD7;
    bool buttonPressedD7;
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
    pb_callback_t destinies;
    bool has_hcsrOn;
    bool hcsrOn;
    bool has_buttonPressedD5;
    bool buttonPressedD5;
    bool has_buttonPressedD2;
    bool buttonPressedD2;
} Msg;

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
    bool has_screenContent;
    ScreenContent screenContent;
    bool has_screenOffsetFrom;
    ScreenOffset screenOffsetFrom;
    bool has_screenOffsetTo;
    ScreenOffset screenOffsetTo;
    bool has_screenClean;
    bool screenClean;
} MsgBack;


/* Helper constants for enums */
#define _MsgBack_ShowType_MIN MsgBack_ShowType_SHOW
#define _MsgBack_ShowType_MAX MsgBack_ShowType_ADDITIONAL
#define _MsgBack_ShowType_ARRAYSIZE ((MsgBack_ShowType)(MsgBack_ShowType_ADDITIONAL+1))


/* Initializer values for message structs */
#define Hello_init_default                       {0, 0, 0, 0, {{NULL}, NULL}}
#define Relay_init_default                       {0, 0}
#define ParsedRemote_init_default                {{{NULL}, NULL}, {{NULL}, NULL}}
#define Msg_init_default                         {0, 0, false, Hello_init_default, {{NULL}, NULL}, {{NULL}, NULL}, 0, {Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default, Relay_init_default}, false, ParsedRemote_init_default, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, {{NULL}, NULL}, false, 0, false, 0, false, 0}
#define ScreenOffset_init_default                {0, 0, 0}
#define ScreenContent_init_default               {0, 0, {{NULL}, NULL}}
#define MsgBack_init_default                     {0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, false, 0, false, _MsgBack_ShowType_MIN, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, {{NULL}, NULL}, false, 0, false, 0, false, 0, false, ScreenContent_init_default, false, ScreenOffset_init_default, false, ScreenOffset_init_default, false, 0}
#define Hello_init_zero                          {0, 0, 0, 0, {{NULL}, NULL}}
#define Relay_init_zero                          {0, 0}
#define ParsedRemote_init_zero                   {{{NULL}, NULL}, {{NULL}, NULL}}
#define Msg_init_zero                            {0, 0, false, Hello_init_zero, {{NULL}, NULL}, {{NULL}, NULL}, 0, {Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero, Relay_init_zero}, false, ParsedRemote_init_zero, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, {{NULL}, NULL}, false, 0, false, 0, false, 0}
#define ScreenOffset_init_zero                   {0, 0, 0}
#define ScreenContent_init_zero                  {0, 0, {{NULL}, NULL}}
#define MsgBack_init_zero                        {0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, false, 0, false, _MsgBack_ShowType_MIN, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, {{NULL}, NULL}, false, 0, {{NULL}, NULL}, {{NULL}, NULL}, false, 0, false, 0, false, 0, false, ScreenContent_init_zero, false, ScreenOffset_init_zero, false, ScreenOffset_init_zero, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define ParsedRemote_remote_tag                  1
#define ParsedRemote_key_tag                     2
#define Hello_versionMajor_tag                   1
#define Hello_versionMinor_tag                   2
#define Hello_versionLast_tag                    3
#define Hello_screenEnabled_tag                  4
#define Hello_settings_tag                       5
#define Relay_id_tag                             1
#define Relay_state_tag                          2
#define ScreenContent_width_tag                  1
#define ScreenContent_height_tag                 2
#define ScreenContent_content_tag                3
#define ScreenOffset_x_tag                       1
#define ScreenOffset_y_tag                       2
#define ScreenOffset_atMs_tag                    3
#define Msg_id_tag                               1
#define Msg_timeseq_tag                          2
#define Msg_hello_tag                            3
#define Msg_irKeyPeriods_tag                     4
#define Msg_debugLogMessage_tag                  5
#define Msg_relayStates_tag                      6
#define Msg_parsedRemote_tag                     7
#define Msg_buttonPressedD7_tag                  8
#define Msg_weight_tag                           9
#define Msg_temp_tag                             10
#define Msg_humidity_tag                         11
#define Msg_pressure_tag                         12
#define Msg_potentiometer_tag                    13
#define Msg_atxState_tag                         14
#define Msg_destinies_tag                        15
#define Msg_hcsrOn_tag                           16
#define Msg_buttonPressedD5_tag                  108
#define Msg_buttonPressedD2_tag                  208
#define MsgBack_id_tag                           1
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
#define MsgBack_introduceYourself_tag            200
#define MsgBack_reboot_tag                       201
#define MsgBack_screenEnable_tag                 202
#define MsgBack_screenContent_tag                300
#define MsgBack_screenOffsetFrom_tag             301
#define MsgBack_screenOffsetTo_tag               302
#define MsgBack_screenClean_tag                  303

/* Struct field encoding specification for nanopb */
#define Hello_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    versionMajor,      1) \
X(a, STATIC,   REQUIRED, INT32,    versionMinor,      2) \
X(a, STATIC,   REQUIRED, INT32,    versionLast,       3) \
X(a, STATIC,   REQUIRED, BOOL,     screenEnabled,     4) \
X(a, CALLBACK, REQUIRED, STRING,   settings,          5)
#define Hello_CALLBACK pb_default_field_callback
#define Hello_DEFAULT NULL

#define Relay_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    id,                1) \
X(a, STATIC,   REQUIRED, BOOL,     state,             2)
#define Relay_CALLBACK NULL
#define Relay_DEFAULT NULL

#define ParsedRemote_FIELDLIST(X, a) \
X(a, CALLBACK, REQUIRED, STRING,   remote,            1) \
X(a, CALLBACK, REQUIRED, STRING,   key,               2)
#define ParsedRemote_CALLBACK pb_default_field_callback
#define ParsedRemote_DEFAULT NULL

#define Msg_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    id,                1) \
X(a, STATIC,   REQUIRED, INT32,    timeseq,           2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  hello,             3) \
X(a, CALLBACK, REPEATED, UINT32,   irKeyPeriods,      4) \
X(a, CALLBACK, OPTIONAL, STRING,   debugLogMessage,   5) \
X(a, STATIC,   REPEATED, MESSAGE,  relayStates,       6) \
X(a, STATIC,   OPTIONAL, MESSAGE,  parsedRemote,      7) \
X(a, STATIC,   OPTIONAL, BOOL,     buttonPressedD7,   8) \
X(a, STATIC,   OPTIONAL, UINT32,   weight,            9) \
X(a, STATIC,   OPTIONAL, FLOAT,    temp,             10) \
X(a, STATIC,   OPTIONAL, FLOAT,    humidity,         11) \
X(a, STATIC,   OPTIONAL, FLOAT,    pressure,         12) \
X(a, STATIC,   OPTIONAL, UINT32,   potentiometer,    13) \
X(a, STATIC,   OPTIONAL, BOOL,     atxState,         14) \
X(a, CALLBACK, REPEATED, UINT32,   destinies,        15) \
X(a, STATIC,   OPTIONAL, BOOL,     hcsrOn,           16) \
X(a, STATIC,   OPTIONAL, BOOL,     buttonPressedD5, 108) \
X(a, STATIC,   OPTIONAL, BOOL,     buttonPressedD2, 208)
#define Msg_CALLBACK pb_default_field_callback
#define Msg_DEFAULT NULL
#define Msg_hello_MSGTYPE Hello
#define Msg_relayStates_MSGTYPE Relay
#define Msg_parsedRemote_MSGTYPE ParsedRemote

#define ScreenOffset_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    x,                 1) \
X(a, STATIC,   REQUIRED, INT32,    y,                 2) \
X(a, STATIC,   REQUIRED, UINT32,   atMs,              3)
#define ScreenOffset_CALLBACK NULL
#define ScreenOffset_DEFAULT NULL

#define ScreenContent_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   width,             1) \
X(a, STATIC,   REQUIRED, UINT32,   height,            2) \
X(a, CALLBACK, REQUIRED, BYTES,    content,           3)
#define ScreenContent_CALLBACK pb_default_field_callback
#define ScreenContent_DEFAULT NULL

#define MsgBack_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, INT32,    id,                1) \
X(a, CALLBACK, OPTIONAL, STRING,   testMsg,           3) \
X(a, STATIC,   OPTIONAL, UINT32,   unixtime,          4) \
X(a, CALLBACK, OPTIONAL, STRING,   textToShow,        5) \
X(a, STATIC,   OPTIONAL, UINT32,   timeMsToShow,      6) \
X(a, STATIC,   OPTIONAL, UENUM,    showType,          7) \
X(a, STATIC,   OPTIONAL, UINT32,   relaysToSwitch,    8) \
X(a, STATIC,   OPTIONAL, BOOL,     relaysToSwitchState,   9) \
X(a, STATIC,   OPTIONAL, BOOL,     atxEnable,        10) \
X(a, STATIC,   OPTIONAL, UINT32,   playMp3,          11) \
X(a, STATIC,   OPTIONAL, UINT32,   volume,           12) \
X(a, STATIC,   OPTIONAL, UINT32,   brightness,       13) \
X(a, STATIC,   OPTIONAL, UINT32,   pwmPin,           14) \
X(a, STATIC,   OPTIONAL, UINT32,   pwmValue,         15) \
X(a, STATIC,   OPTIONAL, UINT32,   pwmPeriod,        16) \
X(a, CALLBACK, OPTIONAL, STRING,   ledValue,         17) \
X(a, STATIC,   OPTIONAL, UINT32,   ledPeriod,        18) \
X(a, CALLBACK, OPTIONAL, STRING,   ledBaseColor,     19) \
X(a, CALLBACK, OPTIONAL, STRING,   ledBlinkColors,   20) \
X(a, STATIC,   OPTIONAL, BOOL,     introduceYourself, 200) \
X(a, STATIC,   OPTIONAL, BOOL,     reboot,          201) \
X(a, STATIC,   OPTIONAL, BOOL,     screenEnable,    202) \
X(a, STATIC,   OPTIONAL, MESSAGE,  screenContent,   300) \
X(a, STATIC,   OPTIONAL, MESSAGE,  screenOffsetFrom, 301) \
X(a, STATIC,   OPTIONAL, MESSAGE,  screenOffsetTo,  302) \
X(a, STATIC,   OPTIONAL, BOOL,     screenClean,     303)
#define MsgBack_CALLBACK pb_default_field_callback
#define MsgBack_DEFAULT NULL
#define MsgBack_screenContent_MSGTYPE ScreenContent
#define MsgBack_screenOffsetFrom_MSGTYPE ScreenOffset
#define MsgBack_screenOffsetTo_MSGTYPE ScreenOffset

extern const pb_msgdesc_t Hello_msg;
extern const pb_msgdesc_t Relay_msg;
extern const pb_msgdesc_t ParsedRemote_msg;
extern const pb_msgdesc_t Msg_msg;
extern const pb_msgdesc_t ScreenOffset_msg;
extern const pb_msgdesc_t ScreenContent_msg;
extern const pb_msgdesc_t MsgBack_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define Hello_fields &Hello_msg
#define Relay_fields &Relay_msg
#define ParsedRemote_fields &ParsedRemote_msg
#define Msg_fields &Msg_msg
#define ScreenOffset_fields &ScreenOffset_msg
#define ScreenContent_fields &ScreenContent_msg
#define MsgBack_fields &MsgBack_msg

/* Maximum encoded size of messages (where known) */
/* Hello_size depends on runtime parameters */
#define Relay_size                               13
/* ParsedRemote_size depends on runtime parameters */
/* Msg_size depends on runtime parameters */
#define ScreenOffset_size                        28
/* ScreenContent_size depends on runtime parameters */
/* MsgBack_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
