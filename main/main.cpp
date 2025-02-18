#include "logger.hpp"
#include "system.hpp"
#include "esp_system.h"

extern "C" void app_main() {
    if (!GetSystem()->initialize()) {
        GetLogger(eLogType::Error)->Log("Failed to initialize system");
        esp_restart();
    }
}
