#include "system.hpp"
#include "logger.hpp"
#include "definition.h"
#include <nvs_flash.h>
#include <esp_netif.h>
#include <esp_mac.h>
#include <esp_chip_info.h>
#include <esp_flash.h>
#include <esp_app_desc.h>
#include <esp_heap_caps.h>

CSystem* CSystem::_instance = nullptr;

CSystem::CSystem() 
{
}

CSystem::~CSystem()
{
    if (_instance) {
        delete _instance;
        _instance = nullptr;
    }
}

CSystem* CSystem::Instance()
{
    if (!_instance) {
        _instance = new CSystem();
    }

    return _instance;
}

bool CSystem::initialize()
{
    GetLogger(eLogType::Info)->Log("Start Initializing System");
    
    GetLogger(eLogType::Info)->Log("System Initialized");
    print_system_info();

    return true;
}

void CSystem::release()
{
}

void CSystem::print_system_info()
{
    GetLogger(eLogType::Info)->Log("System Info");

    // ESP32 specific
    GetLoggerM(eLogType::Info)->Log("----- ESP32 -----");
    auto desc = esp_app_get_description();
    GetLoggerM(eLogType::Info)->Log("Project Name: %s", desc->project_name);
    GetLoggerM(eLogType::Info)->Log("App Version: %s", desc->version);

    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    GetLoggerM(eLogType::Info)->Log("CPU Core(s): %d", chip_info.cores);
    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    GetLoggerM(eLogType::Info)->Log("Revision: %d.%d", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        float flash_size_mb = (float)flash_size / (1024.f * 1024.f);
        GetLoggerM(eLogType::Info)->Log("Flash Size: %g MB (%s)", flash_size_mb, (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    }
    size_t heap_free_size = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    GetLoggerM(eLogType::Info)->Log("Heap Free Size: %d", heap_free_size);

    // network interface
    GetLoggerM(eLogType::Info)->Log("----- Network -----");
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");   // "WIFI_AP_DEF"
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(netif, &ip_info);
    GetLoggerM(eLogType::Info)->Log("IPv4 Address: %d.%d.%d.%d", IP2STR(&ip_info.ip));
    GetLoggerM(eLogType::Info)->Log("Gateway: %d.%d.%d.%d", IP2STR(&ip_info.gw));
    unsigned char mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    GetLoggerM(eLogType::Info)->Log("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
