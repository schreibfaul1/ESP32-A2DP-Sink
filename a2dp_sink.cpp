#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
/*
 * a2dp_sink.cpp.c
 *
 *  Created on: 27.08.2020
 *  Updated on: 10.10.2023
 *      Author: Wolle
 */

#include "a2dp_sink.h"

SemaphoreHandle_t     s_mutex_bt_message;
xQueueHandle          s_bt_app_task_queue      = NULL;;
xTaskHandle           s_bt_app_task_handle     = NULL;
esp_a2d_audio_state_t s_audio_state            = ESP_A2D_AUDIO_STATE_STOPPED;
const char           *s_a2d_conn_state_str[4]  = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
const char           *s_a2d_audio_state_str[3] = {"Suspended", "Stopped", "Started"};
esp_a2d_mct_t         s_audio_type             = 0;
uint8_t              *s_bda                    = NULL;
String                s_BT_sink_name           = "";
i2s_chan_handle_t     s_i2s_tx_handle = {};
i2s_chan_config_t     s_i2s_chan_cfg = {}; // stores I2S channel values
i2s_std_config_t      s_i2s_std_cfg = {};  // stores I2S driver values
char*                 s_chbuf = NULL;
uint8_t               s_vol = 64;

//----------------------------------------------------------------------------------------------------------------------------------------------------
void config_i2s() {

    s_i2s_chan_cfg.id            = (i2s_port_t)I2S_NUM_0;  // I2S_NUM_AUTO, I2S_NUM_0, I2S_NUM_1
    s_i2s_chan_cfg.role          = I2S_ROLE_MASTER;        // I2S controller master role, bclk and lrc signal will be set to output
    s_i2s_chan_cfg.dma_desc_num  = 16;                     // number of DMA buffer
    s_i2s_chan_cfg.dma_frame_num = 512;                    // I2S frame number in one DMA buffer.
    s_i2s_chan_cfg.auto_clear    = true;                   // i2s will always send zero automatically if no data to send
    i2s_new_channel(&s_i2s_chan_cfg, &s_i2s_tx_handle, NULL);

    s_i2s_std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;  // Bits per sample
    s_i2s_std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO;   // I2S channel slot bit-width equals to data bit-width
    s_i2s_std_cfg.slot_cfg.slot_mode      = I2S_SLOT_MODE_STEREO;      // I2S_SLOT_MODE_MONO, I2S_SLOT_MODE_STEREO,
    s_i2s_std_cfg.slot_cfg.slot_mask      = I2S_STD_SLOT_BOTH;         // I2S_STD_SLOT_LEFT, I2S_STD_SLOT_RIGHT
    s_i2s_std_cfg.slot_cfg.ws_width       = I2S_DATA_BIT_WIDTH_16BIT;  // WS signal width (i.e. the number of bclk ticks that ws signal is high)
    s_i2s_std_cfg.slot_cfg.ws_pol         = false;                     // WS signal polarity, set true to enable high lever first
    s_i2s_std_cfg.slot_cfg.bit_shift      = false;                     // Set to enable bit shift in Philips mode
    s_i2s_std_cfg.slot_cfg.msb_right      = true;                      // Set to place right channel data at the MSB in the FIFO
    s_i2s_std_cfg.gpio_cfg.bclk           = I2S_GPIO_UNUSED;           // BCLK, Assignment in setPinout()
    s_i2s_std_cfg.gpio_cfg.din            = I2S_GPIO_UNUSED;           // not used
    s_i2s_std_cfg.gpio_cfg.dout           = I2S_GPIO_UNUSED;           // DOUT, Assignment in setPinout()
    s_i2s_std_cfg.gpio_cfg.mclk           = I2S_GPIO_UNUSED;           // MCLK, Assignment in setPinout()
    s_i2s_std_cfg.gpio_cfg.ws             = I2S_GPIO_UNUSED;           // LRC,  Assignment in setPinout()
    s_i2s_std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
    s_i2s_std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
    s_i2s_std_cfg.gpio_cfg.invert_flags.ws_inv   = false;
    s_i2s_std_cfg.clk_cfg.sample_rate_hz = 44100;
    s_i2s_std_cfg.clk_cfg.clk_src        = I2S_CLK_SRC_DEFAULT;        // Select PLL_F160M as the default source clock
    s_i2s_std_cfg.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_128;      // mclk = sample_rate * 256
    i2s_channel_init_std_mode(s_i2s_tx_handle, &s_i2s_std_cfg);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void set_i2s_pinout(int8_t BCLK, int8_t LRC, int8_t DOUT){ // overwrite default pins

   i2s_std_gpio_config_t gpio_cfg = {};
    gpio_cfg.bclk = (gpio_num_t)BCLK;
    gpio_cfg.din = (gpio_num_t)I2S_GPIO_UNUSED;
    gpio_cfg.dout = (gpio_num_t)DOUT;
    gpio_cfg.mclk = (gpio_num_t)0;
    gpio_cfg.ws = (gpio_num_t)LRC;
  //  i2s_channel_disable(s_i2s_tx_handle);
    i2s_channel_reconfig_std_gpio(s_i2s_tx_handle, &gpio_cfg);
    i2s_channel_enable(s_i2s_tx_handle);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
esp_a2d_audio_state_t get_audio_state() {
    return s_audio_state;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
esp_a2d_mct_t get_audio_type() {
    return s_audio_type;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
bool bt_app_work_dispatch(app_callback_t p_cback, uint16_t event, void *p_params, int param_len){

    if(bt_info){
        xSemaphoreTake(s_mutex_bt_message, 100);
        sprintf(s_chbuf, "event 0x%x, param len %d", event, param_len);
        bt_info(s_chbuf);
        xSemaphoreGive(s_mutex_bt_message);
    }

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
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_app_work_dispatched(app_msg_t *msg){
    if(bt_info){
        xSemaphoreTake(s_mutex_bt_message, 100);
        sprintf(s_chbuf, "event 0x%x, sig 0x%x dispatched", msg->event, msg->sig);
        bt_info(s_chbuf);
        xSemaphoreGive(s_mutex_bt_message);
    }
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
bool bt_app_send_msg(app_msg_t *msg){
    if(bt_info){
        xSemaphoreTake(s_mutex_bt_message, 100);
        sprintf(s_chbuf, "send msg: event 0x%x, sig 0x%x", msg->event, msg->sig);
        bt_info(s_chbuf);
        xSemaphoreGive(s_mutex_bt_message);
    }
    if (msg == NULL) {
        return false;
    }

    if (xQueueSend(s_bt_app_task_queue, msg, 10 / portTICK_RATE_MS) != pdTRUE) {
        log_e("xQueue send failed");
        return false;
    }
    return true;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_app_task_handler(void *arg){
    app_msg_t msg;
    for (;;) {
        if(pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (portTickType)portMAX_DELAY)){
            switch (msg.sig) {
                case APP_SIG_WORK_DISPATCH:
                    if(bt_info){
                        xSemaphoreTake(s_mutex_bt_message, 100);
                        sprintf(s_chbuf, "task handler: APP_SIG_WORK_DISPATCH sig: %d", msg.sig);
                        bt_info(s_chbuf);
                        xSemaphoreGive(s_mutex_bt_message);
                    }
                    bt_app_work_dispatched(&msg);
                    break;
                default:
                    if(bt_info){
                        xSemaphoreTake(s_mutex_bt_message, 100);
                        sprintf(s_chbuf, "task handler: unhandled sig: 0%x", msg.sig);
                        bt_info(s_chbuf);
                        xSemaphoreGive(s_mutex_bt_message);
                    }
                    break;
            } // switch (msg.sig)

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_app_task_start_up(void){
    s_bt_app_task_queue = xQueueCreate(10, sizeof(app_msg_t));
    xTaskCreate(bt_app_task_handler, "BtAppT", 2048, NULL, configMAX_PRIORITIES - 3, &s_bt_app_task_handle);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param){
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);
    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    if(bt_info){
        xSemaphoreTake(s_mutex_bt_message, 100);
        sprintf(s_chbuf, "metadata: attr_text= %s", attr_text);
        bt_info(s_chbuf);
        xSemaphoreGive(s_mutex_bt_message);
    }
    rc->meta_rsp.attr_text = attr_text;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param){
    switch (event) {
        case ESP_AVRC_CT_METADATA_RSP_EVT:
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                bt_info("ESP_AVRC_CT_METADATA_RSP_EVT");
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_app_alloc_meta_buffer(param);
            bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_CONNECTION_STATE_EVT:
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                bt_info("ESP_AVRC_CT_CONNECTION_STATE_EVT");
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                bt_info("ESP_AVRC_CT_PASSTHROUGH_RSP_EVT");
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                bt_info("ESP_AVRC_CT_CHANGE_NOTIFY_EVT");
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                bt_info("ESP_AVRC_CT_REMOTE_FEATURES_EVT");
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t));
            break;
        }
        default:
            log_e("Invalid AVRC event: %d", event);
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_av_hdl_a2d_evt(uint16_t event, void *p_param){
    esp_a2d_cb_param_t *a2d = NULL;
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        if(bt_info){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "ESP_A2D_CONNECTION_STATE_EVT %i", event);
            bt_info(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
                s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        if(bt_info){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "ESP_A2D_AUDIO_STATE_EVT %i", event);
            bt_info(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "A2DP audio state: %s", s_a2d_audio_state_str[a2d->audio_stat.state]);
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
        s_audio_state = a2d->audio_stat.state;
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            ;
        }
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
        if(bt_info){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "ESP_A2D_AUDIO_CFG_EVT %i", event);
            bt_info(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
        esp_a2d_cb_param_t *esp_a2d_callback_param = (esp_a2d_cb_param_t *)(p_param);
        s_audio_type = esp_a2d_callback_param->audio_cfg.mcc.type;
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if(bt_info){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "a2dp audio_cfg, codec type %d", a2d->audio_cfg.mcc.type);
            bt_info(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
        // for now only SBC stream is supported
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            s_i2s_std_cfg.clk_cfg.sample_rate_hz = 16000;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
                s_i2s_std_cfg.clk_cfg.sample_rate_hz = 32000;
            } else if (oct0 & (0x01 << 5)) {
                s_i2s_std_cfg.clk_cfg.sample_rate_hz = 44100;
            } else if (oct0 & (0x01 << 4)) {
                s_i2s_std_cfg.clk_cfg.sample_rate_hz = 48000;
            }
            i2s_channel_disable(s_i2s_tx_handle);
            i2s_channel_reconfig_std_slot(s_i2s_tx_handle, &s_i2s_std_cfg.slot_cfg);
            i2s_channel_enable(s_i2s_tx_handle);
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "configure audio player [%02x-%02x-%02x-%02x]", a2d->audio_cfg.mcc.cie.sbc[0], a2d->audio_cfg.mcc.cie.sbc[1],
                                                                                 a2d->audio_cfg.mcc.cie.sbc[2], a2d->audio_cfg.mcc.cie.sbc[3]);
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            if(bt_state){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "audio player configured, samplerate = %lu", s_i2s_std_cfg.clk_cfg.sample_rate_hz);
                bt_state(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
        }
        break;
    }
    default:
        log_e("unhandled evt 0x%x", event);
        break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_av_new_track(){
    //Register notifications and request metadata
    esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_GENRE);
    esp_avrc_ct_send_register_notification_cmd(1, ESP_AVRC_RN_TRACK_CHANGE, 0);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t event_parameter){
    switch (event_id) {
        case ESP_AVRC_RN_TRACK_CHANGE:
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "ESP_AVRC_RN_TRACK_CHANGE %d",event_id);
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_av_new_track();
            break;
        default:
            log_e("unhandled evt %d", event_id);
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_av_hdl_avrc_evt(uint16_t event, void *p_param){
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);
    switch (event) {
        case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
            uint8_t *bda = rc->conn_stat.remote_bda;
            s_bda = rc->conn_stat.remote_bda;
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                                                                  rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            if (rc->conn_stat.connected) {
                bt_av_new_track();
            }
            break;
        }
        case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "AVRC passthrough rsp: key_code 0x%x, key_state %d", rc->psth_rsp.key_code, rc->psth_rsp.key_state);
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            break;
        }
        case ESP_AVRC_CT_METADATA_RSP_EVT: {
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "AVRC metadata rsp: attribute id 0x%lx, %s", (uint32_t)rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            free(rc->meta_rsp.attr_text);
            break;
        }
        case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "AVRC event notification: %u", rc->change_ntf.event_id);
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_av_notify_evt_handler(rc->change_ntf.event_id, rc->change_ntf.event_parameter);
            break;
        }
        case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "AVRC remote features 0x%lx", rc->rmt_feats.feat_mask);
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            break;
        }
        default:
            log_e("unhandled evt %d", event);
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_av_hdl_stack_evt(uint16_t event, void *p_param){
    switch (event) {
        case BT_APP_EVT_STACK_UP: {
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "av_hdl_stack_evt %s","BT_APP_EVT_STACK_UP");
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
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
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            break;
        }
        default:
            log_e("unhandled evt %d",event);
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param){
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "ESP_A2D_CONNECTION_STATE_EVT");
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t));
            break;
        case ESP_A2D_AUDIO_STATE_EVT:
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "ESP_A2D_AUDIO_STATE_EVT");
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            s_audio_state = param->audio_stat.state;
            bt_app_work_dispatch(bt_av_hdl_a2d_evt,event, param, sizeof(esp_a2d_cb_param_t));
            break;
        case ESP_A2D_AUDIO_CFG_EVT: {

            bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t));
            break;
        }
        case ESP_A2D_PROF_STATE_EVT:{ // indicate a2dp init&deinit complete
            if(bt_info){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "ESP_A2D_PROF_STATE_EVT");
                bt_info(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
            break;
        }

        default:
            log_e("Invalid A2DP event: %d", event);
            break;
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len) {

    uint32_t s32;
    int16_t s16_1;
    int16_t s16_2;

    size_t i2s_bytes_written = 0;

    for(uint32_t i = 0; i < len; i += 4){

        s16_1 = data[i + 0] + (data[i + 1] << 8);
        s16_2 = data[i + 2] + (data[i + 3] << 8);

        s16_1 *= s_vol >> 6;
        s16_2 *= s_vol >> 6;

        s32 = s16_1 + (s16_2 << 16);

        size_t bytesWritten = 0;
        if(i2s_channel_write(s_i2s_tx_handle, (const char*)&s32, sizeof(uint32_t), &i2s_bytes_written, portMAX_DELAY)){
             log_e("i2s_write has failed");
        }
        i2s_bytes_written += bytesWritten;
    }
    if (i2s_bytes_written < len){
       log_e("Timeout: not all bytes were written to I2S");
    }
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
bool a2dp_sink_deinit(){
    esp_err_t res;

//    res = i2s_driver_uninstall(s_i2s_port);
//    if(res != ESP_OK){if(bt_info) bt_info("Failed to uninstall i2s"); goto exit;}

    res = esp_a2d_sink_deinit();
    if(res != ESP_OK){if(bt_info) bt_info("Failed to deinit a2d sink"); goto exit;}
    else{
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "bt controller deinit okay");
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
    }

    if(s_bda){
        res = esp_a2d_sink_disconnect(s_bda);
        if(res != ESP_OK){if(bt_info) bt_info("Failed to disconnect a2d sink"); goto exit;}
        else{
            if(bt_state){
                xSemaphoreTake(s_mutex_bt_message, 100);
                sprintf(s_chbuf, "a2d sink disconnected");
                bt_state(s_chbuf);
                xSemaphoreGive(s_mutex_bt_message);
            }
        }
    }

    res = esp_bluedroid_deinit();
    if(res !=  ESP_OK){if(bt_info) bt_info("Failed to deinit bluedroid"); goto exit;}
    else{
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "bluedroid deinit okay");
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
    }

    res = esp_bluedroid_disable();
    if(res != ESP_OK){log_e("Failed to disable bluedroid"); goto exit;}
    else{
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "bluedroid disabled");
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
    }


    if(bt_state) bt_state("BT deinit okay");
    if(s_chbuf){free(s_chbuf); s_chbuf = NULL;}
    vSemaphoreDelete(s_mutex_bt_message);
    return true;

exit:
    if(s_chbuf){free(s_chbuf); s_chbuf = NULL;}
    vSemaphoreDelete(s_mutex_bt_message);
    return false;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------
bool a2dp_sink_init(String deviceName){


    esp_err_t res;
    if(s_chbuf){free(s_chbuf); s_chbuf = NULL;}
    s_chbuf = (char*)malloc(512);
    s_mutex_bt_message = xSemaphoreCreateMutex();

    s_BT_sink_name = deviceName;
    if(bt_state){
        xSemaphoreTake(s_mutex_bt_message, 100);
        sprintf(s_chbuf, "Device name will be set to '%s'", s_BT_sink_name.c_str());
        bt_state(s_chbuf);
        xSemaphoreGive(s_mutex_bt_message);
    }
    if(!btStart()) {log_e("Failed to initialize controller"); return false;}
    else{
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "bt controller initialized");
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
    }
  
    res = esp_bluedroid_init();
    if(res != ESP_OK) {log_e("Failed to initialize bluedroid"); return false;}
    else{
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "bluedroid initialized");
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
    }

    res = esp_bluedroid_enable();
    if(res != ESP_OK) {log_e("Failed to enable bluedroid"); return false;}
    else{
        if(bt_state){
            xSemaphoreTake(s_mutex_bt_message, 100);
            sprintf(s_chbuf, "bluedroid enabled");
            bt_state(s_chbuf);
            xSemaphoreGive(s_mutex_bt_message);
        }
    }

    bt_app_task_start_up(); // create application task

    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0);

//    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED; // Set default parameters for Legacy Pairing
//    esp_bt_pin_code_t pin_code;                         // Use fixed pin code
//    pin_code[0] = '1';
//    pin_code[1] = '2';
//    pin_code[2] = '3';
//    pin_code[3] = '4';
//    esp_bt_gap_set_pin(pin_type, 4, pin_code);


    config_i2s();

    return true;
}
