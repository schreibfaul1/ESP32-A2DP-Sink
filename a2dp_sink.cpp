/*
 * a2dp_sink.cpp.c
 *
 *  Created on: 26.08.2020
 *      Author: wolle
 */


#include "a2dp_sink.h"

static xQueueHandle          s_bt_app_task_queue      = NULL;;
static xTaskHandle           s_bt_app_task_handle     = NULL;
static esp_a2d_audio_state_t s_audio_state            = ESP_A2D_AUDIO_STATE_STOPPED;
static const char           *s_a2d_conn_state_str[4]  = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
static const char           *s_a2d_audio_state_str[3] = {"Suspended", "Stopped", "Started"};
static esp_a2d_mct_t         s_audio_type             = 0;
static uint8_t              *s_bda                    = NULL;
static String                s_BT_sink_name           = "";
static i2s_port_t            s_i2s_port = I2S_NUM_0;
static i2s_config_t          s_i2s_config;
static i2s_pin_config_t      s_pin_config;

//---------------------------------------------------------------------------------------------------------------------
void config_i2s() {

    // setup default i2s config
    s_i2s_config.mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX);
    s_i2s_config.sample_rate = 44100;
    s_i2s_config.bits_per_sample = (i2s_bits_per_sample_t) 16;
    s_i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    s_i2s_config.communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
    s_i2s_config.intr_alloc_flags = 0; // default interrupt priority
    s_i2s_config.dma_buf_count = 8;
    s_i2s_config.dma_buf_len = 64;
    s_i2s_config.use_apll = false;

    i2s_driver_install(s_i2s_port, &s_i2s_config, 0, NULL);

    // setup default pins
    s_pin_config.bck_io_num = 27;                 // BCLK
    s_pin_config.ws_io_num = 26;                  // LRC
    s_pin_config.data_out_num = 25;               // DOUT
    s_pin_config.data_in_num = I2S_PIN_NO_CHANGE; // DIN

    i2s_set_pin(s_i2s_port, &s_pin_config);
}
//---------------------------------------------------------------------------------------------------------------------
void set_i2s_pinout(int8_t BCLK, int8_t LRC, int8_t DOUT){ // overwrite default pins

    s_pin_config.bck_io_num   = BCLK;              // BCLK
    s_pin_config.ws_io_num    = LRC;               // LRC
    s_pin_config.data_out_num = DOUT;              // DOUT
    s_pin_config.data_in_num  = I2S_PIN_NO_CHANGE; // DIN

    i2s_set_pin(s_i2s_port, &s_pin_config);
}
//---------------------------------------------------------------------------------------------------------------------
esp_a2d_audio_state_t get_audio_state() {
    return s_audio_state;
}
//---------------------------------------------------------------------------------------------------------------------
esp_a2d_mct_t get_audio_type() {
    return s_audio_type;
}
//---------------------------------------------------------------------------------------------------------------------
bool bt_app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len){
    log_i("event 0x%x, param len %d", event, param_len);

    app_msg_t msg;
    memset(&msg, 0, sizeof(app_msg_t));

    msg.sig = APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_work_dispatched(app_msg_t *msg){
    log_i("event 0x%x, sig 0x%x", msg->event, msg->sig);
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}
//---------------------------------------------------------------------------------------------------------------------
bool bt_app_send_msg(app_msg_t *msg){
    log_i("event 0x%x, sig 0x%x", msg->event, msg->sig);
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        log_e("xQueue send failed");
        return false;
    }
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_task_handler(void *arg){
    app_msg_t msg;
    for (;;) {
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (portTickType)portMAX_DELAY)) {
            log_i("enent 0x%x, sig 0x%x",msg.event, msg.sig);
            switch (msg.sig) {
            case APP_SIG_WORK_DISPATCH:
                log_w("APP_SIG_WORK_DISPATCH sig: %d", msg.sig);
                bt_app_work_dispatched(&msg);
                break;
            default:
                log_e("unhandled sig: 0%x", msg.sig);
                break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_task_start_up(void){
    s_bt_app_task_queue = xQueueCreate(10, sizeof(app_msg_t));
    xTaskCreate(bt_app_task_handler, "BtAppT", 2048, NULL, configMAX_PRIORITIES - 3, &s_bt_app_task_handle);
}
//---------------------------------------------------------------------------------------------------------------------
void  bt_app_task_shut_down(void){
    if (s_bt_app_task_handle) {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue) {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param){
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);
    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    log_d("attr_text= %s", attr_text);
    rc->meta_rsp.attr_text = attr_text;
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param){
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        log_d("ESP_AVRC_CT_METADATA_RSP_EVT");
        bt_app_alloc_meta_buffer(param);
        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        log_d("ESP_AVRC_CT_CONNECTION_STATE_EVT");
        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
        log_d("ESP_AVRC_CT_PASSTHROUGH_RSP_EVT");
        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        log_d("ESP_AVRC_CT_CHANGE_NOTIFY_EVT");
        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        log_d("ESP_AVRC_CT_REMOTE_FEATURES_EVT");
        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
        break;
    }
    default:
        log_e("Invalid AVRC event: %d", event);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_av_hdl_a2d_evt(uint16_t event, void *p_param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        log_d("ESP_A2D_CONNECTION_STATE_EVT %i", event);
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        log_i("A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
             s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        log_d("ESP_A2D_AUDIO_STATE_EVT %i", event);
        a2d = (esp_a2d_cb_param_t *)(p_param);
        log_i("A2DP audio state: %s", s_a2d_audio_state_str[a2d->audio_stat.state]);
        s_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            ;
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
        log_d("ESP_A2D_AUDIO_CFG_EVT %i", event);
        esp_a2d_cb_param_t *esp_a2d_callback_param = (esp_a2d_cb_param_t *)(p_param);
        s_audio_type = esp_a2d_callback_param->audio_cfg.mcc.type;
        a2d = (esp_a2d_cb_param_t *)(p_param);
        log_i("a2dp audio_cfg, codec type %d", a2d->audio_cfg.mcc.type);
        // for now only SBC stream is supported
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            s_i2s_config.sample_rate = 16000;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
                s_i2s_config.sample_rate = 32000;
            } else if (oct0 & (0x01 << 5)) {
                s_i2s_config.sample_rate = 44100;
            } else if (oct0 & (0x01 << 4)) {
                s_i2s_config.sample_rate = 48000;
            }

            i2s_set_clk(s_i2s_port, s_i2s_config.sample_rate, s_i2s_config.bits_per_sample, (i2s_channel_t)2);

            log_i("configure audio player %x-%x-%x-%x\n",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            log_i("audio player configured, samplerate=%d", s_i2s_config.sample_rate);
        }
        break;
    }
    default:
        log_e("unhandled evt 0x%x", event);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_av_new_track(){
    //Register notifications and request metadata
    esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_GENRE);
    esp_avrc_ct_send_register_notification_cmd(1, ESP_AVRC_RN_TRACK_CHANGE, 0);
}
//---------------------------------------------------------------------------------------------------------------------
void bt_av_notify_evt_handler(uint8_t event_id, uint32_t event_parameter){
    switch (event_id) {
    case ESP_AVRC_RN_TRACK_CHANGE:
        log_d("ESP_AVRC_RN_TRACK_CHANGE %d",event_id);
        bt_av_new_track();
        break;
    default:
        log_e("unhandled evt %d", event_id);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_av_hdl_avrc_evt(uint16_t event, void *p_param){
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);
    switch (event) {
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        s_bda = rc->conn_stat.remote_bda;
        log_i("AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            bt_av_new_track();
        }
        break;
    }
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        log_i("AVRC passthrough rsp: key_code 0x%x, key_state %d", rc->psth_rsp.key_code, rc->psth_rsp.key_state);
        break;
    }
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGI(BT_AV_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
        free(rc->meta_rsp.attr_text);
        break;
    }
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        log_i("AVRC event notification: %d, param: %d", rc->change_ntf.event_id, rc->change_ntf.event_parameter);
        bt_av_notify_evt_handler(rc->change_ntf.event_id, rc->change_ntf.event_parameter);
        break;
    }
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        log_i("AVRC remote features %x", rc->rmt_feats.feat_mask);
        break;
    }
    default:
        log_e("unhandled evt %d", event);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_av_hdl_stack_evt(uint16_t event, void *p_param){
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        log_i("av_hdl_stack_evt %s","BT_APP_EVT_STACK_UP");
        /* set up device name */
        esp_bt_dev_set_device_name(s_BT_sink_name.c_str());

        /* initialize A2DP sink */
        esp_a2d_register_callback(bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_sink_init();

        /* initialize AVRCP controller */
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        break;
    }
    default:
        log_e("unhandled evt %d",event);
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        log_d("ESP_A2D_CONNECTION_STATE_EVT");
        bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_STATE_EVT:
        log_d("ESP_A2D_AUDIO_STATE_EVT");
        s_audio_state = param->audio_stat.state;
        bt_app_work_dispatch(bt_av_hdl_a2d_evt,event, param, sizeof(esp_a2d_cb_param_t));
        break;
    case ESP_A2D_AUDIO_CFG_EVT: {
        log_d("ESP_A2D_AUDIO_CFG_EVT");
        bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t));
        break;
    }
    default:
        log_e("Invalid A2DP event: %d");
        break;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len) {
   size_t i2s_bytes_written;
   if (i2s_write(s_i2s_port,(void*) data, len, &i2s_bytes_written, portMAX_DELAY)!=ESP_OK){
      log_e("i2s_write has failed");
   }

   if (i2s_bytes_written<len){
      log_e("Timeout: not all bytes were written to I2S");
   }
}
//---------------------------------------------------------------------------------------------------------------------
bool a2dp_sink_deinit(){
    esp_err_t res;
    bt_app_task_shut_down();

    res = i2s_driver_uninstall(s_i2s_port);
    if(res != ESP_OK){log_e("Failed to uninstall i2s"); return false;}

    res = esp_a2d_sink_deinit();
    if(res != ESP_OK){log_e("Failed to deinit a2d sink"); return false;}

    if(s_bda){
        res = esp_a2d_sink_disconnect(s_bda);
        if(res != ESP_OK){log_e("Failed to disconnect a2d sink"); return false;}
    }

    res = esp_bluedroid_deinit();
    if(res !=  ESP_OK){log_e("Failed to deinit bluedroid"); return false;}

    res = esp_bluedroid_disable();
    if(res != ESP_OK){log_e("Failed to disable bluedroid"); return false;}

    return true;
}
//---------------------------------------------------------------------------------------------------------------------
bool a2dp_sink_init(String deviceName){
    esp_err_t res;

    s_BT_sink_name = deviceName;
    log_i("Device name will be set to '%s'", s_BT_sink_name.c_str());

//    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));




    if(!btStart()) {log_e("Failed to initialize controller"); return false;}
    else log_i("controller initialized");

//    res = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
//    if(res != ESP_OK) {log_e("Enable controller failed"); return false;}

    res = esp_bluedroid_init();
    if(res != ESP_OK) {log_e("Failed to initialize bluedroid"); return false;}
    else log_i("bluedroid initialized");

    res = esp_bluedroid_enable();
    if(res != ESP_OK) {log_e("Failed to enable bluedroid"); return false;}
    else log_i("bluedroid enabled");

    bt_app_task_start_up(); // create application task

    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0);

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED; // Set default parameters for Legacy Pairing
    esp_bt_pin_code_t pin_code;                         // Use fixed pin code
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    config_i2s();

    return true;
}


