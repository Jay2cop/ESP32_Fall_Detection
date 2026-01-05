#pragma once

// Copy this file to include/config.h and fill in your secrets.
// Never commit the populated config.h file to source control.

#define DEVICE_NAME "ESP32 Fall Detector"
#define ENABLE_DEBUG_LOGS 1

// Webhook / alert configuration (e.g., IFTTT Webhooks service)
#define ALERT_WEBHOOK_BASE_URL "http://maker.ifttt.com"
#define ALERT_WEBHOOK_TRIGGER_PATH "/trigger/"
#define ALERT_WEBHOOK_AUTH_PATH "/with/key/"
#define ALERT_FALL_EVENT "Fall_detect"
#define ALERT_RESET_EVENT "Fall_reset"
#define ALERT_COLLECTION_EVENT "Fall_collection"
#define ALERT_WEBHOOK_KEY "REPLACE_WITH_IFTTT_KEY"
