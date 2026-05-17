#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

/*******************************************************
 *                Constants / Configuration
 *******************************************************/
#define BASE_STATION_NODE_ID  23768
#define MAX_BUNDLES_IN_RAM    100
#define BUNDLE_PAYLOAD_SIZE   1024
#define MAX_PEERS             10
#define ESPNOW_CHANNEL        36      // 5GHz UNII-1, 5180 MHz, no DFS
#define BEACON_INTERVAL_MS    1000
#define TX_INTERVAL_MS        1000
#define PEER_TIMEOUT_MS       15000

#define PKT_TYPE_BEACON  0x01
#define PKT_TYPE_BUNDLE  0x02

// ESP-NOW v2.0 max payload is 1470 bytes.
// Our largest possible bundle packet: 1 (type) + bundle_header + 1024 (payload) ≈ 1053 bytes.
// All ESP32-C5 nodes on IDF 6 negotiate v2.0 automatically.

/*******************************************************
 *                Structs
 *******************************************************/

// BPv7-inspired bundle (unchanged from ESP-WIFI-MESH version)
typedef struct __attribute__((packed)) {
    uint32_t creation_time;
    uint32_t sequence_number;
    uint32_t lifetime;
    uint16_t source_node;
    uint16_t dest_node;
    uint16_t report_to_node;
    uint16_t prev_node;
    bool     request_delivery_report;
    bool     is_telemetry;
    uint8_t  hop_limit;
    uint8_t  hop_count;
    size_t   payload_len;
    uint8_t  payload[BUNDLE_PAYLOAD_SIZE];
} dtn_bundle_t;

typedef struct {
    dtn_bundle_t bundle;
    bool is_empty;
    bool forwarded;
} ram_bundle_t;

typedef struct __attribute__((packed)) {
    uint8_t  pkt_type;       // PKT_TYPE_BEACON
    uint16_t node_id;
    uint32_t timestamp_ms;
} beacon_pkt_t;

typedef struct {
    uint8_t  mac[6];
    uint16_t node_id;
    uint32_t last_seen_ms;
    bool     active;
    int32_t  clock_offset_ms; // peer_clock - bs_clock at last beacon; valid when active
} peer_entry_t;

// Queue item for recv_cb → rx_process_task
#define RX_BUF_MAX 1472
typedef struct {
    uint8_t  src_mac[6];
    int8_t   rssi;
    uint16_t len;
    uint8_t  data[RX_BUF_MAX];
} rx_item_t;

/*******************************************************
 *                Globals
 *******************************************************/
static const char *TAG = "dtn_now";

static ram_bundle_t      bundle_store[MAX_BUNDLES_IN_RAM];
static uint32_t          local_sequence_counter = 0;
static peer_entry_t      peer_list[MAX_PEERS];
static int               peer_count = 0;
static SemaphoreHandle_t data_mutex;
static QueueHandle_t     rx_queue;

// Static TX buffer — written only from bundle_tx_task (single writer, no race)
static uint8_t tx_pkt_buf[1 + sizeof(dtn_bundle_t)];

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/*******************************************************
 *                Helper Functions
 *******************************************************/
static inline uint32_t now_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

uint16_t get_my_node_id(void) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return (uint16_t)((mac[4] << 8) | mac[5]);
}

static void init_bundle_store(void) {
    for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
        bundle_store[i].is_empty  = true;
        bundle_store[i].forwarded = false;
        memset(&bundle_store[i].bundle, 0, sizeof(dtn_bundle_t));
    }
    ESP_LOGI(TAG, "Bundle store initialized");
}

// Register MAC with ESP-NOW peer table. Idempotent.
static bool register_espnow_peer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peer = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac, 6);
    return esp_now_add_peer(&peer) == ESP_OK;
}

// Add new peer or refresh last_seen on existing one.
// Returns true if this was a genuinely new peer (triggers forwarded reset).
// Must NOT be called while holding data_mutex.
static bool add_or_refresh_peer(const uint8_t *mac, uint16_t node_id, int8_t rssi) {
    uint32_t ts = now_ms();
    bool is_new = false;

    xSemaphoreTake(data_mutex, portMAX_DELAY);

    for (int i = 0; i < peer_count; i++) {
        if (memcmp(peer_list[i].mac, mac, 6) == 0) {
            bool was_inactive = !peer_list[i].active;
            peer_list[i].last_seen_ms = ts;
            peer_list[i].active = true;
            if (was_inactive) {
                // Peer returned after timeout — reset forwarded so it gets everything stored
                for (int j = 0; j < MAX_BUNDLES_IN_RAM; j++) {
                    if (!bundle_store[j].is_empty) bundle_store[j].forwarded = false;
                }
                is_new = true;
            }
            xSemaphoreGive(data_mutex);
            if (is_new) {
                register_espnow_peer(mac);  // re-add: was deleted on timeout
                ESP_LOGI(TAG, "Peer returned: node=%u mac=%02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
                         node_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi);
            }
            return is_new;
        }
    }

    if (peer_count < MAX_PEERS) {
        memcpy(peer_list[peer_count].mac, mac, 6);
        peer_list[peer_count].node_id      = node_id;
        peer_list[peer_count].last_seen_ms = ts;
        peer_list[peer_count].active       = true;
        peer_count++;
        is_new = true;
        // New peer: reset forwarded so they receive all stored bundles
        for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
            if (!bundle_store[i].is_empty) bundle_store[i].forwarded = false;
        }
    } else {
        ESP_LOGW(TAG, "Peer list full, ignoring node %u", node_id);
    }

    xSemaphoreGive(data_mutex);

    if (is_new) {
        register_espnow_peer(mac);
        ESP_LOGI(TAG, "New peer: node=%u mac=%02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
                 node_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], rssi);
    }
    return is_new;
}

/*******************************************************
 *                ESP-NOW Callbacks
 *******************************************************/

// Recv callback: runs in WiFi task context. Copy to queue and return fast.
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
    if (data_len <= 0 || data_len > RX_BUF_MAX) return;
    rx_item_t item;
    memcpy(item.src_mac, info->src_addr, 6);
    item.rssi = (int8_t)info->rx_ctrl->rssi;
    item.len  = (uint16_t)data_len;
    memcpy(item.data, data, data_len);
    if (xQueueSend(rx_queue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "RX queue full, packet dropped");
    }
}

/*******************************************************
 *                RX Processing Task
 *******************************************************/
static void rx_process_task(void *arg) {
    uint16_t my_id = get_my_node_id();
    rx_item_t item;

    while (1) {
        if (xQueueReceive(rx_queue, &item, portMAX_DELAY) != pdTRUE) continue;
        if (item.len < 1) continue;

        uint8_t pkt_type = item.data[0];

        // --- Beacon ---
        if (pkt_type == PKT_TYPE_BEACON && item.len >= (uint16_t)sizeof(beacon_pkt_t)) {
            beacon_pkt_t *beacon = (beacon_pkt_t *)item.data;
            add_or_refresh_peer(item.src_mac, beacon->node_id, item.rssi);

            // BS: update per-peer clock offset and emit @NET:
            if (my_id == BASE_STATION_NODE_ID) {
                int32_t offset = (int32_t)beacon->timestamp_ms - (int32_t)now_ms();
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                for (int k = 0; k < peer_count; k++) {
                    if (peer_list[k].node_id == beacon->node_id) {
                        peer_list[k].clock_offset_ms = offset;
                        break;
                    }
                }
                xSemaphoreGive(data_mutex);
                printf("@NET:%u:%u:%d\n", beacon->node_id, my_id, (int)item.rssi);
            }

        // --- Bundle ---
        } else if (pkt_type == PKT_TYPE_BUNDLE) {
            size_t hdr_size = sizeof(dtn_bundle_t) - BUNDLE_PAYLOAD_SIZE;
            if ((size_t)item.len < 1 + hdr_size) continue;

            dtn_bundle_t *incoming = (dtn_bundle_t *)(item.data + 1);

            xSemaphoreTake(data_mutex, portMAX_DELAY);

            // Dedup by (source_node, creation_time, sequence_number)
            bool already_have = false;
            for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                if (!bundle_store[i].is_empty) {
                    dtn_bundle_t *b = &bundle_store[i].bundle;
                    if (b->source_node    == incoming->source_node &&
                        b->creation_time  == incoming->creation_time &&
                        b->sequence_number == incoming->sequence_number) {
                        already_have = true;
                        break;
                    }
                }
            }

            if (already_have) {
                xSemaphoreGive(data_mutex);
                ESP_LOGI(TAG, "@DEDUP src:%u seq:%" PRIu32,
                         incoming->source_node, incoming->sequence_number);
                continue;
            }

            // Find empty slot
            int slot = -1;
            for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                if (bundle_store[i].is_empty) { slot = i; break; }
            }

            if (slot < 0) {
                xSemaphoreGive(data_mutex);
                ESP_LOGW(TAG, "Bundle store full! Dropping src:%u seq:%" PRIu32,
                         incoming->source_node, incoming->sequence_number);
                continue;
            }

            // Store bundle
            size_t copy_size = (size_t)item.len - 1;
            memcpy(&bundle_store[slot].bundle, incoming, copy_size);
            bundle_store[slot].is_empty  = false;
            // BS marks forwarded=true immediately so the TX task won't re-broadcast
            bundle_store[slot].forwarded = (my_id == BASE_STATION_NODE_ID);

            // Snapshot fields needed outside mutex
            uint32_t btime      = incoming->creation_time;
            bool     btelemetry = incoming->is_telemetry;
            uint16_t bsrc       = incoming->source_node;
            uint16_t bprev      = incoming->prev_node;
            uint32_t bseq       = incoming->sequence_number;
            uint8_t  bhops      = incoming->hop_count;
            int32_t  src_clock_offset = 0;
            if (my_id == BASE_STATION_NODE_ID) {
                for (int k = 0; k < peer_count; k++) {
                    if (peer_list[k].node_id == bsrc) {
                        src_clock_offset = peer_list[k].clock_offset_ms;
                        break;
                    }
                }
            }

            int stored_count = 0;
            for (int k = 0; k < MAX_BUNDLES_IN_RAM; k++) {
                if (!bundle_store[k].is_empty) stored_count++;
            }

            // Rover: generate DTN_RX ack telemetry bundle (inside mutex while we have space)
            if (my_id != BASE_STATION_NODE_ID && !btelemetry) {
                for (int j = 0; j < MAX_BUNDLES_IN_RAM; j++) {
                    if (bundle_store[j].is_empty) {
                        dtn_bundle_t *ack = &bundle_store[j].bundle;
                        ack->creation_time   = now_ms();
                        ack->sequence_number = local_sequence_counter++;
                        ack->lifetime        = 10000;
                        ack->source_node     = my_id;
                        ack->dest_node       = 0;
                        ack->prev_node       = my_id;
                        ack->hop_limit       = 5;
                        ack->hop_count       = 0;
                        ack->is_telemetry    = true;
                        ack->payload_len     = snprintf(
                            (char *)ack->payload, BUNDLE_PAYLOAD_SIZE,
                            "@DTN_RX:%u:%u:%" PRIu32 ":%u:%u",
                            bsrc, bprev, bseq, my_id, bhops);
                        bundle_store[j].is_empty  = false;
                        bundle_store[j].forwarded = false;
                        break;
                    }
                }
            }

            // Save payload text for BS printing (short copy, avoids using bundle_store after release)
            char payload_text[BUNDLE_PAYLOAD_SIZE];
            size_t plen = 0;
            if (my_id == BASE_STATION_NODE_ID && btelemetry) {
                plen = incoming->payload_len < (BUNDLE_PAYLOAD_SIZE - 1)
                     ? incoming->payload_len : (BUNDLE_PAYLOAD_SIZE - 1);
                memcpy(payload_text, incoming->payload, plen);
                payload_text[plen] = '\0';
            }

            xSemaphoreGive(data_mutex);

            // Logging / serial output (no mutex needed)
            if (my_id == BASE_STATION_NODE_ID) {
                // Adjust for clock skew: rover clocks start independently at boot.
                // src_clock_offset = rover_ms - bs_ms at last beacon; corrects creation_time.
                int32_t latency = (int32_t)(now_ms() - btime) + src_clock_offset;
                if (latency < 0) latency = 0;
                printf("@METRIC:%u:%" PRIu32 ":%d:%d\n",
                       bsrc, bseq, bhops, (int)latency);
                if (btelemetry) {
                    printf("%s\n", payload_text);
                } else {
                    printf("@DTN_RX:%u:%u:%" PRIu32 ":%u:%u\n",
                           bsrc, bprev, bseq, my_id, bhops);
                }
                // Free the slot (BS doesn't store bundles)
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                bundle_store[slot].is_empty = true;
                xSemaphoreGive(data_mutex);
            } else {
                ESP_LOGI(TAG, "STORED src:%u seq:%" PRIu32 " hops:%u | store:%d/%d",
                         bsrc, bseq, bhops, stored_count, MAX_BUNDLES_IN_RAM);
            }
        }
    }
}

/*******************************************************
 *                Beacon Task
 *******************************************************/
static void beacon_task(void *arg) {
    uint16_t my_id = get_my_node_id();

    while (1) {
        beacon_pkt_t pkt = {
            .pkt_type     = PKT_TYPE_BEACON,
            .node_id      = my_id,
            .timestamp_ms = now_ms(),
        };
        esp_now_send(BROADCAST_MAC, (uint8_t *)&pkt, sizeof(pkt));
        vTaskDelay(pdMS_TO_TICKS(BEACON_INTERVAL_MS));
    }
}

/*******************************************************
 *                Bundle TX Task
 *******************************************************/
static void bundle_tx_task(void *arg) {
    uint16_t my_id = get_my_node_id();
    int bundle_gen_timer  = 0;
    int store_dump_timer  = 0;
    int bs_heartbeat_timer = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
        uint32_t ts = now_ms();

        // --- Expire timed-out peers ---
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        for (int i = 0; i < peer_count; i++) {
            if (peer_list[i].active && (ts - peer_list[i].last_seen_ms) > PEER_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Peer node=%u timed out", peer_list[i].node_id);
                peer_list[i].active = false;
                esp_now_del_peer(peer_list[i].mac);
            }
        }
        xSemaphoreGive(data_mutex);

        // --- BS self-heartbeat for visualizer (every 5s) ---
        if (my_id == BASE_STATION_NODE_ID) {
            bs_heartbeat_timer++;
            if (bs_heartbeat_timer >= 5) {
                bs_heartbeat_timer = 0;
                printf("@NET:%u:0:0\n", my_id);
            }
        }

        // --- Generate data bundle every 10s ---
        bundle_gen_timer++;
        if (bundle_gen_timer >= 10) {
            bundle_gen_timer = 0;
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                if (bundle_store[i].is_empty) {
                    dtn_bundle_t *b = &bundle_store[i].bundle;
                    b->creation_time         = ts;
                    b->sequence_number       = local_sequence_counter++;
                    b->lifetime              = 300000;
                    b->source_node           = my_id;
                    b->dest_node             = 0;
                    b->prev_node             = my_id;
                    b->hop_limit             = 5;
                    b->hop_count             = 0;
                    b->is_telemetry          = false;
                    b->request_delivery_report = false;
                    b->payload_len           = snprintf((char *)b->payload, BUNDLE_PAYLOAD_SIZE,
                                                        "Data from %u", my_id);
                    bundle_store[i].is_empty  = false;
                    bundle_store[i].forwarded = false;
                    ESP_LOGW(TAG, "ORIGINATED Bundle seq:%" PRIu32, b->sequence_number);
                    break;
                }
            }
            xSemaphoreGive(data_mutex);
        }

        // --- Periodic store dump (every 10s) ---
        store_dump_timer++;
        if (store_dump_timer >= 10) {
            store_dump_timer = 0;
            int total = 0;
            uint16_t seen_src[10] = {0};
            int      seen_cnt[10] = {0};
            int unique = 0;
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                if (bundle_store[i].is_empty) continue;
                total++;
                uint16_t src = bundle_store[i].bundle.source_node;
                bool found = false;
                for (int j = 0; j < unique; j++) {
                    if (seen_src[j] == src) { seen_cnt[j]++; found = true; break; }
                }
                if (!found && unique < 10) { seen_src[unique] = src; seen_cnt[unique] = 1; unique++; }
            }
            xSemaphoreGive(data_mutex);
            if (total == 0) {
                ESP_LOGI(TAG, "@STORE: empty");
            } else {
                char dump_buf[128] = {0};
                int pos = 0;
                for (int j = 0; j < unique; j++) {
                    pos += snprintf(dump_buf + pos, sizeof(dump_buf) - pos,
                                    "%u(%d) ", seen_src[j], seen_cnt[j]);
                }
                ESP_LOGI(TAG, "@STORE: %d bundles | %s", total, dump_buf);
            }
        }

        // --- Epidemic forwarding ---
        // Snapshot active peer MACs (brief mutex hold), then send without holding mutex.
        uint8_t  peer_macs[MAX_PEERS][6];
        uint16_t peer_ids[MAX_PEERS];
        int n_active = 0;

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        for (int i = 0; i < peer_count; i++) {
            if (peer_list[i].active) {
                memcpy(peer_macs[n_active], peer_list[i].mac, 6);
                peer_ids[n_active] = peer_list[i].node_id;
                n_active++;
            }
        }
        xSemaphoreGive(data_mutex);

        for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            if (bundle_store[i].is_empty) { xSemaphoreGive(data_mutex); continue; }

            dtn_bundle_t *b = &bundle_store[i].bundle;

            // TTL aging
            if (b->lifetime <= (uint32_t)TX_INTERVAL_MS) {
                ESP_LOGW(TAG, "Bundle TTL expired src:%u seq:%" PRIu32,
                         b->source_node, b->sequence_number);
                bundle_store[i].is_empty = true;
                xSemaphoreGive(data_mutex);
                continue;
            }
            b->lifetime -= TX_INTERVAL_MS;

            // Hop limit
            if (b->hop_count >= b->hop_limit) {
                ESP_LOGW(TAG, "Bundle hop limit exceeded src:%u seq:%" PRIu32,
                         b->source_node, b->sequence_number);
                bundle_store[i].is_empty = true;
                xSemaphoreGive(data_mutex);
                continue;
            }

            if (bundle_store[i].forwarded || n_active == 0) {
                xSemaphoreGive(data_mutex);
                continue;
            }

            // Build TX packet (mutated copy — stored bundle is never modified)
            uint16_t orig_prev = b->prev_node;
            size_t bundle_actual = sizeof(dtn_bundle_t) - BUNDLE_PAYLOAD_SIZE + b->payload_len;
            size_t pkt_size = 1 + bundle_actual;
            tx_pkt_buf[0] = PKT_TYPE_BUNDLE;
            dtn_bundle_t *tx_copy = (dtn_bundle_t *)(tx_pkt_buf + 1);
            memcpy(tx_copy, b, bundle_actual);
            tx_copy->prev_node = my_id;
            tx_copy->hop_count = b->hop_count + 1;

            xSemaphoreGive(data_mutex);

            // Send to all active peers (no mutex held during send)
            bool sent_to_anyone = false;
            for (int j = 0; j < n_active; j++) {
                if (peer_ids[j] == orig_prev) continue; // don't echo back to prev hop

                esp_err_t err = esp_now_send(peer_macs[j], tx_pkt_buf, pkt_size);
                ESP_LOGI(TAG, "TX src:%u seq:%" PRIu32 " -> node:%u %s",
                         tx_copy->source_node, tx_copy->sequence_number,
                         peer_ids[j], err == ESP_OK ? "OK" : "FAIL");
                if (err == ESP_OK) sent_to_anyone = true;
            }

            if (sent_to_anyone) {
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                if (!bundle_store[i].is_empty) bundle_store[i].forwarded = true;
                xSemaphoreGive(data_mutex);
            }
        }
    }
}

/*******************************************************
 *                app_main
 *******************************************************/
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_wifi_start());

    // ESP32-C5 is dual-band; lock to 5GHz after start, then set channel
    ESP_ERROR_CHECK(esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY));

    // Fixed channel — all nodes must match
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Broadcast peer required for beacon TX
    esp_now_peer_info_t bcast = {
        .channel = ESPNOW_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(bcast.peer_addr, BROADCAST_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&bcast));

    uint16_t my_id = get_my_node_id();
    if (my_id == BASE_STATION_NODE_ID) {
        ESP_LOGI(TAG, "I am the Base Station (ID=%u) on channel %d (5GHz)", my_id, ESPNOW_CHANNEL);
    } else {
        ESP_LOGI(TAG, "I am a Rover (ID=%u) on channel %d (5GHz)", my_id, ESPNOW_CHANNEL);
    }

    init_bundle_store();
    data_mutex = xSemaphoreCreateMutex();
    rx_queue   = xQueueCreate(4, sizeof(rx_item_t));

    xTaskCreate(rx_process_task, "rx_proc",  4096, NULL, 6, NULL);
    xTaskCreate(beacon_task,     "beacon",   2048, NULL, 5, NULL);
    xTaskCreate(bundle_tx_task,  "tx_loop",  4096, NULL, 4, NULL);
}
