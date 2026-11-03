#include "touch_display.h"
#include "sdkconfig.h"
#if CONFIG_HEATING_TOUCH_DISPLAY
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ek79007.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_ldo_regulator.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lvgl.h"
#include "config.h"
#include "outputs.h"
#include "ntp.h"
#include "sensors.h"
#include "stove.h"
#include "mqtt.h"
#include "ethernet.h"
#include "cJSON.h"
#define W 1024
#define H 600
#define POINTS 289
static const char *TAG="touch_display";
static void (*get_status)(heating_status_t *);
static esp_lcd_panel_handle_t panel;
static esp_lcd_touch_handle_t touch;
static lv_obj_t *temps[6],*state_label,*header,*mode_note,*mode_buttons[6],*settings,*chart,*history_note;
static lv_chart_series_t *series[6];
static const char *names[]={"Aussen","Raum","Kessel","Vorlauf","Boiler","Kachelofen"};
static const char *mode_names[]={"Aus","Frostschutz","Automatik","Nur Heizung","Nur Boiler","Urlaub"};
static const char *mode_keys[]={"off","frost","auto","heating","dhw","holiday"};
static const heating_mode_t mode_values[]={MODE_OFF,MODE_FROST,MODE_AUTO,MODE_HEATING_ONLY,MODE_DHW_ONLY,MODE_HOLIDAY};
static const uint32_t colors[]={0x246fbd,0x207e71,0xbb493d,0x8754a3,0xa26910,0xcf6025};
static int pending_mode=-1;
static unsigned history_hours=1;
static bool history_dirty=true,visible[6]={true,true,true,true,true,true};
static uint32_t last_mode_write;
static uint32_t ticks(void){return (uint32_t)(esp_timer_get_time()/1000);}
static lv_obj_t *label(lv_obj_t *parent,const char *value,int x,int y,const lv_font_t *font){
 lv_obj_t *o=lv_label_create(parent);lv_label_set_text(o,value);lv_obj_set_pos(o,x,y);lv_obj_set_style_text_font(o,font,0);return o;
}
static void flush(lv_display_t *d,const lv_area_t *area,uint8_t *pixels){
 /* Ohne DMA2D ist draw_bitmap synchron; der Puffer ist danach wieder frei. */
 esp_err_t e=esp_lcd_panel_draw_bitmap(panel,area->x1,area->y1,area->x2+1,area->y2+1,pixels);
 if(e!=ESP_OK)ESP_LOGE(TAG,"Bilduebertragung: %s",esp_err_to_name(e));
 lv_display_flush_ready(d);
}
static void touch_read(lv_indev_t *indev,lv_indev_data_t *data){
 (void)indev;data->state=LV_INDEV_STATE_RELEASED;
 if(!touch)return;
 esp_err_t e=esp_lcd_touch_read_data(touch);
 if(e!=ESP_OK){
  static uint32_t last_error;
  uint32_t now=ticks();
  if(!last_error||now-last_error>=10000){ESP_LOGW(TAG,"Touch lesen: %s",esp_err_to_name(e));last_error=now;}
  return;
 }
 uint16_t x=0,y=0,strength=0;uint8_t count=0;
 if(esp_lcd_touch_get_coordinates(touch,&x,&y,&strength,&count,1)&&count&&x<W&&y<H){
  data->point.x=x;data->point.y=y;data->state=LV_INDEV_STATE_PRESSED;
 }
}
static void mode_click(lv_event_t *event){if(ticks()-last_mode_write<2000)return;pending_mode=(int)(intptr_t)lv_event_get_user_data(event);last_mode_write=ticks();}
static void show_series(lv_event_t *e){int n=(int)(intptr_t)lv_event_get_user_data(e);visible[n]=lv_obj_has_state(lv_event_get_target_obj(e),LV_STATE_CHECKED);lv_chart_hide_series(chart,series[n],!visible[n]);history_dirty=true;}
static void history_range(lv_event_t *e){history_hours=(unsigned)(uintptr_t)lv_event_get_user_data(e);history_dirty=true;}
static void build_ui(void){
 lv_obj_t *screen=lv_screen_active();lv_obj_set_style_bg_color(screen,lv_color_hex(0xeef5f6),0);
 lv_obj_set_style_text_color(screen,lv_color_hex(0x183d48),0);
 header=label(screen,"HEIZUNGSSTEUERUNG  |  Display startet",20,12,&lv_font_montserrat_20);
 lv_obj_t *tabs=lv_tabview_create(screen);lv_obj_set_pos(tabs,0,48);lv_obj_set_size(tabs,W,H-48);
 lv_tabview_set_tab_bar_position(tabs,LV_DIR_BOTTOM);lv_tabview_set_tab_bar_size(tabs,64);
 lv_obj_set_style_text_font(lv_tabview_get_tab_bar(tabs),&lv_font_montserrat_20,0);
 lv_obj_t *overview=lv_tabview_add_tab(tabs,"Uebersicht"),*modes=lv_tabview_add_tab(tabs,"Betriebsart");
 lv_obj_t *history=lv_tabview_add_tab(tabs,"Verlauf"),*options=lv_tabview_add_tab(tabs,"Einstellungen");
 lv_obj_t *pages[]={overview,modes,history,options};for(int i=0;i<4;i++){lv_obj_set_style_pad_all(pages[i],12,0);lv_obj_remove_flag(pages[i],LV_OBJ_FLAG_SCROLLABLE);}
 for(int i=0;i<6;i++){
  lv_obj_t *card=lv_obj_create(overview);lv_obj_set_pos(card,(i%3)*240,(i/3)*215);lv_obj_set_size(card,230,200);
  lv_obj_remove_flag(card,LV_OBJ_FLAG_SCROLLABLE);lv_obj_set_style_border_color(card,lv_color_hex(0xd2e1e4),0);
  label(card,names[i],0,4,&lv_font_montserrat_20);temps[i]=label(card,"--.-- C",0,70,&lv_font_montserrat_40);
 }
 state_label=label(overview,"",730,4,&lv_font_montserrat_20);lv_obj_set_width(state_label,245);
 mode_note=label(modes,"Aktuelle Betriebsart wird geladen",12,10,&lv_font_montserrat_20);
 for(int i=0;i<6;i++){
  lv_obj_t *b=lv_button_create(modes);mode_buttons[i]=b;lv_obj_set_pos(b,12+(i%3)*325,70+(i/3)*145);lv_obj_set_size(b,305,120);
  lv_obj_add_flag(b,LV_OBJ_FLAG_CHECKABLE);lv_obj_add_event_cb(b,mode_click,LV_EVENT_CLICKED,(void *)(intptr_t)i);
  lv_obj_set_style_bg_color(b,lv_color_hex(0x207e71),LV_STATE_CHECKED);lv_obj_t *l=lv_label_create(b);lv_label_set_text(l,mode_names[i]);lv_obj_set_style_text_font(l,&lv_font_montserrat_28,0);lv_obj_center(l);
 }
 label(modes,"Betriebsart wie im Web ohne Anmeldung waehlbar.",12,395,&lv_font_montserrat_20);
 chart=lv_chart_create(history);lv_obj_set_pos(chart,40,65);lv_obj_set_size(chart,925,280);
 lv_chart_set_type(chart,LV_CHART_TYPE_LINE);lv_chart_set_point_count(chart,61);lv_chart_set_range(chart,LV_CHART_AXIS_PRIMARY_Y,0,100);
 for(int i=0;i<6;i++){
  series[i]=lv_chart_add_series(chart,lv_color_hex(colors[i]),LV_CHART_AXIS_PRIMARY_Y);lv_chart_set_all_value(chart,series[i],LV_CHART_POINT_NONE);
  lv_obj_t *cb=lv_checkbox_create(history);lv_obj_set_pos(cb,5+i*165,365);lv_checkbox_set_text(cb,names[i]);lv_obj_add_state(cb,LV_STATE_CHECKED);lv_obj_set_style_text_font(cb,&lv_font_montserrat_20,0);lv_obj_set_style_text_color(cb,lv_color_hex(colors[i]),0);lv_obj_add_event_cb(cb,show_series,LV_EVENT_VALUE_CHANGED,(void *)(intptr_t)i);
 }
 history_note=label(history,"Warte auf Messpunkte",12,4,&lv_font_montserrat_20);
 for(int i=0;i<3;i++){unsigned hours[]={1,6,24};lv_obj_t *b=lv_button_create(history);lv_obj_set_pos(b,650+i*105,0);lv_obj_set_size(b,95,40);lv_obj_t *l=lv_label_create(b);lv_label_set_text_fmt(l,"%u Std",hours[i]);lv_obj_center(l);lv_obj_add_event_cb(b,history_range,LV_EVENT_CLICKED,(void *)(uintptr_t)hours[i]);}
 label(history,"Links: Beginn des Zeitraums  |  rechts: jetzt  |  Neustart loescht Verlauf",12,410,&lv_font_montserrat_20);
 settings=label(options,"",15,12,&lv_font_montserrat_20);lv_obj_set_width(settings,950);
 label(options,"Geschuetzte Einstellungen bitte nach Anmeldung auf der Webseite aendern.",15,410,&lv_font_montserrat_20);
}
static void update_history(void){
 static int values[6][POINTS];
 unsigned span=history_hours*3600,bin_s=history_hours==1?60:history_hours==6?120:300;int count=(int)(span/bin_s)+1;lv_chart_set_point_count(chart,count);for(int i=0;i<6;i++)for(int j=0;j<POINTS;j++)values[i][j]=LV_CHART_POINT_NONE;
 int64_t now=esp_timer_get_time()/1000;cJSON *local=sensors_history_json(now,span),*p;
 cJSON_ArrayForEach(p,cJSON_GetObjectItemCaseSensitive(local,"points")){
  cJSON *age=cJSON_GetObjectItemCaseSensitive(p,"age_s"),*v=cJSON_GetObjectItemCaseSensitive(p,"temperatures");
  if(!cJSON_IsNumber(age)||age->valuedouble<0||age->valuedouble>span)continue;
  int bin=count-1-(int)(age->valuedouble/bin_s);
  for(int i=0;i<5;i++){cJSON *n=cJSON_GetArrayItem(v,i);if(cJSON_IsNumber(n))values[i][bin]=(int)lround(n->valuedouble*100);}
 }cJSON_Delete(local);
 char *json=stove_history_json(now);cJSON *stove=json?cJSON_Parse(json):NULL;free(json);
 cJSON_ArrayForEach(p,cJSON_GetObjectItemCaseSensitive(stove,"points")){
  cJSON *age=cJSON_GetObjectItemCaseSensitive(p,"age_s"),*v=cJSON_GetObjectItemCaseSensitive(p,"temperature");
  if(cJSON_IsNumber(age)&&age->valuedouble>=0&&age->valuedouble<=span&&cJSON_IsNumber(v))values[5][count-1-(int)(age->valuedouble/bin_s)]=(int)lround(v->valuedouble*100);
 }cJSON_Delete(stove);
 int low=10000,high=-10000;
 for(int i=0;i<6;i++)for(int j=0;j<count;j++){lv_chart_set_value_by_id(chart,series[i],j,values[i][j]);if(visible[i]&&values[i][j]!=LV_CHART_POINT_NONE){if(values[i][j]<low)low=values[i][j];if(values[i][j]>high)high=values[i][j];}}
 if(low<=high){lv_chart_set_range(chart,LV_CHART_AXIS_PRIMARY_Y,low-100,high+100);lv_label_set_text_fmt(history_note,"%u Std | Skala %.2f bis %.2f C",history_hours,(low-100)/100.0,(high+100)/100.0);}
 if(low>high)lv_label_set_text(history_note,"Keine sichtbaren Messwerte");
 lv_chart_refresh(chart);history_dirty=false;
}
static void update_status(void){
 heating_status_t s;get_status(&s);float position;bool timeout;outputs_get(&s.outputs,&position,&timeout);heating_config_t c;config_get(&c);sensor_snapshot_t v;sensors_get(&v);
 stove_state_t st;stove_get(NULL,&st,esp_timer_get_time()/1000);
 for(int i=0;i<6;i++){bool valid=i<5?v.value[i].valid:st.valid;float t=i<5?v.value[i].celsius:st.celsius;
  if(valid)lv_label_set_text_fmt(temps[i],"%.2f C",(double)t);else lv_label_set_text(temps[i],"--.-- C");
 }
 time_t now=time(NULL);struct tm tm;localtime_r(&now,&tm);
 lv_label_set_text_fmt(header,"HEIZUNGSSTEUERUNG  |  %02d:%02d %s | ETH %s MQTT %s",tm.tm_hour,tm.tm_min,now>1700000000?"":"(Zeit fehlt)",ethernet_has_ip()?"OK":"--",mqtt_connected()?"OK":"--");
 lv_label_set_text_fmt(state_label,"%s\n\nBrenner: %s\n\nHeizpumpe: %s\n\nBoilerpumpe: %s\n\nMischer: %s\n\nRestwaerme: %s",config_mode_name(c.mode),s.outputs.burner?"Ein":"Aus",s.outputs.heating_pump?"Ein":"Aus",s.outputs.boiler_pump?"Ein":"Aus",s.outputs.mixer==MIX_OPEN?"Auf":s.outputs.mixer==MIX_CLOSE?"Zu":"Stop",s.rest_heat_active?"Aktiv":"Aus");
 for(int i=0;i<6;i++){if(c.mode==mode_values[i])lv_obj_add_state(mode_buttons[i],LV_STATE_CHECKED);else lv_obj_remove_state(mode_buttons[i],LV_STATE_CHECKED);}
 if(pending_mode>=0){char json[48],reason[100];snprintf(json,sizeof(json),"{\"mode\":\"%s\"}",mode_keys[pending_mode]);pending_mode=-1;esp_err_t e=config_mode_update_json(json,reason,sizeof(reason));lv_label_set_text(mode_note,e==ESP_OK?"Betriebsart gespeichert":reason);}
 else lv_label_set_text_fmt(mode_note,"Aktuell: %s",config_mode_name(c.mode));
 lv_label_set_text_fmt(settings,"Heizkurve\nSteilheit %.1f  |  Vorlauf bei 0 C: %.1f C\nVorlauf min/max: %.1f / %.1f C\n\nBoiler\nSoll %.1f C  |  Hysterese %.1f C\n\nMischer\nVollstaendige Laufzeit: %u Sekunden\n\nTemperaturen und Einstellungen entsprechen der Webseite.",(double)c.curve_slope,(double)c.curve_zero,(double)c.flow_min,(double)c.flow_max,(double)c.boiler_target,(double)c.boiler_hysteresis,(unsigned)c.mixer_travel_s);
}
static esp_err_t hardware_init(void){
 if(heap_caps_get_total_size(MALLOC_CAP_SPIRAM)<2*1024*1024){ESP_LOGE(TAG,"PSRAM fehlt; Anzeige bleibt aus");return ESP_ERR_NO_MEM;}
 i2c_master_bus_handle_t i2c;i2c_master_bus_config_t ic={.i2c_port=I2C_NUM_0,.sda_io_num=7,.scl_io_num=8,.clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7,.flags.enable_internal_pullup=true};
 esp_err_t e=i2c_new_master_bus(&ic,&i2c);if(e!=ESP_OK)return e;
 /* Touch-C requires its panel power/reset sequence BEFORE DSI initialization.
    Waveshare BSP esp32_p4_platform, pinned source 80402cdb, 7_INCH_C branch. */
 if((e=i2c_master_probe(i2c,0x45,100))!=ESP_OK){ESP_LOGE(TAG,"Displayversorgung-I2C 0x45 fehlt: %s",esp_err_to_name(e));return e;}
 i2c_master_dev_handle_t control;i2c_device_config_t dc={.dev_addr_length=I2C_ADDR_BIT_LEN_7,.device_address=0x45,.scl_speed_hz=100000};
 if((e=i2c_master_bus_add_device(i2c,&dc,&control))!=ESP_OK)return e;
 static const uint8_t startup[][2]={{0x95,0x11},{0x95,0x17},{0x96,0x00},{0x96,0xff}};
 for(unsigned i=0;i<sizeof(startup)/sizeof(startup[0]);i++){
  e=i2c_master_transmit(control,startup[i],sizeof(startup[i]),100);
  if(e!=ESP_OK){ESP_LOGE(TAG,"Display-Einschaltfolge Schritt %u: %s",i+1,esp_err_to_name(e));break;}
 }
 i2c_master_bus_rm_device(control);if(e!=ESP_OK)return e;
 ESP_LOGI(TAG,"Touch-C Displayversorgung freigegeben; Hintergrundlicht eingeschaltet");
 esp_ldo_channel_handle_t ldo;esp_ldo_channel_config_t power={.chan_id=3,.voltage_mv=2500};
 e=esp_ldo_acquire_channel(&power,&ldo);if(e!=ESP_OK)return e;
 esp_lcd_dsi_bus_handle_t bus;esp_lcd_dsi_bus_config_t bc=EK79007_PANEL_BUS_DSI_2CH_CONFIG();
 if((e=esp_lcd_new_dsi_bus(&bc,&bus))!=ESP_OK)return e;
 esp_lcd_panel_io_handle_t io;esp_lcd_dbi_io_config_t dbi=EK79007_PANEL_IO_DBI_CONFIG();
 if((e=esp_lcd_new_panel_io_dbi(bus,&dbi,&io))!=ESP_OK)return e;
 esp_lcd_dpi_panel_config_t dpi=EK79007_1024_600_PANEL_60HZ_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);dpi.flags.use_dma2d=false;
 ek79007_vendor_config_t vendor={.mipi_config={.dsi_bus=bus,.dpi_config=&dpi}};
 esp_lcd_panel_dev_config_t pc={.reset_gpio_num=-1,.rgb_ele_order=LCD_RGB_ELEMENT_ORDER_RGB,.bits_per_pixel=16,.vendor_config=&vendor};
 if((e=esp_lcd_new_panel_ek79007(io,&pc,&panel))!=ESP_OK)return e;
 if((e=esp_lcd_panel_reset(panel))!=ESP_OK)return e;
 if((e=esp_lcd_panel_init(panel))!=ESP_OK)return e;
 uint8_t addr=i2c_master_probe(i2c,0x5d,100)==ESP_OK?0x5d:0x14;
 if(i2c_master_probe(i2c,addr,100)==ESP_OK){esp_lcd_panel_io_handle_t tio;esp_lcd_panel_io_i2c_config_t tc=ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();tc.dev_addr=addr;tc.scl_speed_hz=100000;
  if((e=esp_lcd_new_panel_io_i2c(i2c,&tc,&tio))!=ESP_OK)return e;
  esp_lcd_touch_io_gt911_config_t extra={.dev_addr=addr};/* GT911 reports the Touch-C coordinates rotated 180 degrees relative to
     this panel scan orientation. Inclusive maxima keep edge coordinates valid. */
  esp_lcd_touch_config_t tcfg={.x_max=W-1,.y_max=H-1,.rst_gpio_num=-1,.int_gpio_num=-1,.flags={.mirror_x=1,.mirror_y=1},.driver_data=&extra};
  e=esp_lcd_touch_new_i2c_gt911(tio,&tcfg,&touch);if(e!=ESP_OK)ESP_LOGW(TAG,"Touchinitialisierung: %s",esp_err_to_name(e));
 }else ESP_LOGW(TAG,"Kein GT911 an 0x5d/0x14; Anzeige ohne Touch");
 lv_init();lv_tick_set_cb(ticks);lv_display_t *d=lv_display_create(W,H);if(!d)return ESP_ERR_NO_MEM;
 lv_display_set_color_format(d,LV_COLOR_FORMAT_RGB565);lv_display_set_flush_cb(d,flush);
 void *buffer=heap_caps_aligned_alloc(64,W*40*2,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);if(!buffer)return ESP_ERR_NO_MEM;
 lv_display_set_buffers(d,buffer,NULL,W*40*2,LV_DISPLAY_RENDER_MODE_PARTIAL);
 if(touch){lv_indev_t *input=lv_indev_create();if(!input)return ESP_ERR_NO_MEM;lv_indev_set_type(input,LV_INDEV_TYPE_POINTER);lv_indev_set_display(input,d);lv_indev_set_read_cb(input,touch_read);}
 return ESP_OK;
}
static void display_task(void *unused){
 (void)unused;esp_err_t e=hardware_init();if(e!=ESP_OK){ESP_LOGE(TAG,"Anzeige nicht gestartet: %s; Regler laeuft weiter",esp_err_to_name(e));vTaskDelete(NULL);return;}
 build_ui();update_status();update_history();uint32_t last=ticks(),history=last;
 ESP_LOGI(TAG,"Touch-C gestartet, Touch %s",touch?"bereit":"fehlt");
 for(;;){uint32_t now=ticks();if(now-last>=1000){update_status();last=now;}if(history_dirty||now-history>=30000){update_history();history=now;}
  uint32_t delay=lv_timer_handler();if(delay<10)delay=10;if(delay>30)delay=30;vTaskDelay(pdMS_TO_TICKS(delay));}
}
void touch_display_start(void (*reader)(heating_status_t *)){
 get_status=reader;if(xTaskCreate(display_task,"touch_display",12288,NULL,1,NULL)!=pdPASS)ESP_LOGE(TAG,"Keine Displaytask");
}
#else
void touch_display_start(void (*reader)(heating_status_t *)){(void)reader;}
#endif
