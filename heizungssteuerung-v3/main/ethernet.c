#include "ethernet.h"
#include "board.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include <stdatomic.h>
static atomic_bool has_ip;
static void event(void *arg,esp_event_base_t base,int32_t id,void *data){
 (void)arg;(void)data;
 if(base==IP_EVENT&&id==IP_EVENT_ETH_GOT_IP){atomic_store(&has_ip,true);ESP_LOGI("ethernet","DHCP-Adresse erhalten");}
 if((base==ETH_EVENT&&id==ETHERNET_EVENT_DISCONNECTED)||(base==IP_EVENT&&id==IP_EVENT_ETH_LOST_IP))atomic_store(&has_ip,false);
}
bool ethernet_has_ip(void){return atomic_load(&has_ip);}
esp_err_t ethernet_init(void){
 ESP_ERROR_CHECK(esp_netif_init());ESP_ERROR_CHECK(esp_event_loop_create_default());
 esp_netif_config_t netcfg=ESP_NETIF_DEFAULT_ETH();esp_netif_t *net=esp_netif_new(&netcfg);
 if(!net)return ESP_ERR_NO_MEM;
 eth_mac_config_t mac_cfg=ETH_MAC_DEFAULT_CONFIG();eth_esp32_emac_config_t emac=ETH_ESP32_EMAC_DEFAULT_CONFIG();
 emac.smi_gpio.mdc_num=PIN_ETH_MDC;emac.smi_gpio.mdio_num=PIN_ETH_MDIO;
 emac.clock_config.rmii.clock_mode=EMAC_CLK_EXT_IN;emac.clock_config.rmii.clock_gpio=PIN_ETH_CLOCK;
 /* P4 RMII-Gruppe: Default 49,34,35,28,29,30 stimmt mit Waveshare überein. */
 eth_phy_config_t phy_cfg=ETH_PHY_DEFAULT_CONFIG();phy_cfg.phy_addr=CONFIG_HEATING_PHY_ADDR;phy_cfg.reset_gpio_num=PIN_ETH_RESET;
 esp_eth_mac_t *mac=esp_eth_mac_new_esp32(&emac,&mac_cfg);esp_eth_phy_t *phy=esp_eth_phy_new_ip101(&phy_cfg);
 if(!mac||!phy)return ESP_ERR_NO_MEM;
 esp_eth_config_t cfg=ETH_DEFAULT_CONFIG(mac,phy);esp_eth_handle_t handle=NULL;
 ESP_ERROR_CHECK(esp_eth_driver_install(&cfg,&handle));
 esp_eth_netif_glue_handle_t glue=esp_eth_new_netif_glue(handle);if(!glue)return ESP_ERR_NO_MEM;
 ESP_ERROR_CHECK(esp_netif_attach(net,glue));
 ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT,ESP_EVENT_ANY_ID,event,NULL));
 ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_ETH_GOT_IP,event,NULL));
 ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_ETH_LOST_IP,event,NULL));
 return esp_eth_start(handle);
}
