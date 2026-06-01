#include <string.h>
#include <stdlib.h>
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
#include "driver/uart.h"
#include "driver/gpio.h"
// i2c_master.h removed , LCD uses bit-bang I2C via GPIO to bypass driver pin checks
#include "esp_rom_sys.h"

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
#define PKT_TYPE_ANTIPKT 0x03

#define MAX_ANTIPKTS          50
#define ANTIPKT_LIFETIME_MS   (120 * 1000)
#define MAX_BS_DELIVERED      200

// ESP-NOW v2.0 max payload is 1470 bytes.
// Our largest possible bundle packet: 1 (type) + bundle_header + 1024 (payload) ≈ 1053 bytes.
// All ESP32-C5 nodes on IDF 6 negotiate v2.0 automatically.

// Host interface (Jetson ↔ ESP32 UART1)
// Frame: [SOF:0xAA][CMD:1][LEN_LO][LEN_HI][PAYLOAD:LEN][CRC16_LO][CRC16_HI]
// CRC-16/CCITT over [CMD, LEN_LO, LEN_HI, PAYLOAD...]
#define HOST_UART_PORT        UART_NUM_0  // bench/USB; swap to UART_NUM_1 + GPIO6/7 for Jetson
#define HOST_UART_TX_PIN      UART_PIN_NO_CHANGE
#define HOST_UART_RX_PIN      UART_PIN_NO_CHANGE
#define HOST_UART_BAUD        115200
#define HOST_UART_RX_BUF      2048
#define HOST_SOF              0xAA
#define HOST_MAX_PAYLOAD      1060      // headroom above sizeof(dtn_bundle_t)
#define HOST_PULL_INTERVAL_MS 1000      // pull from Jetson store every 1s when peers active
// Jetson -> ESP32
#define HOST_CMD_TX_BUNDLE    0x01      // payload: raw dtn_bundle_t bytes
#define HOST_CMD_QUERY_STATUS 0x02      // payload: none
#define HOST_CMD_QUERY_PEERS  0x03      // payload: none
// ESP32 -> Jetson
#define HOST_CMD_PULL_REQ     0x10      // payload: none; reply = BUNDLE_DATA or NO_BUNDLES
#define HOST_CMD_ACK          0x20      // payload: 1 byte (0=ok, 1=error/full)
#define HOST_CMD_STATUS_RESP  0x21      // payload: host_status_t
#define HOST_CMD_PEERS_RESP   0x22      // payload: n × host_peer_entry_t
// Both directions
#define HOST_CMD_BUNDLE_DATA  0x11      // payload: raw dtn_bundle_t bytes
#define HOST_CMD_NO_BUNDLES   0x12      // payload: none
#define HOST_CMD_ANTIPKT_NOTIFY 0x13   // ESP32 -> Jetson: drop bundle; payload: antipkt_id_t (10 bytes)
#define HOST_CMD_BUNDLE_PUSH    0x14   // ESP32 -> Jetson: store received bundle; payload: raw dtn_bundle_t bytes

// UI hardware , adjust GPIO numbers to match your board
#define UI_LED_PIN          GPIO_NUM_10  // status LED, all nodes          (J1 pin 11)
#define UI_BTN_PIN          GPIO_NUM_9   // pushbutton, BS only             (J1 pin 10) 
#define LCD_SDA_PIN         GPIO_NUM_7    // I2C SDA, BS only               (J1 pin 8)
#define LCD_SCL_PIN         GPIO_NUM_8    // I2C SCL, BS only               (J1 pin 9)

#define LCD_I2C_ADDR        0x27         // PCF8574 backpack address
#define LCD_COLS            16
#define UI_TASK_HZ          50           // ui_update_task rate
#define UI_LCD_EVERY        25           // refresh LCD every N task cycles (25 × 20ms = 500ms)
#define UI_BTN_DEBOUNCE     3            // consistent reads required (3 × 20ms = 60ms)

/*******************************************************
 *                Structs
 *******************************************************/

// bp7 inspired bundle format 
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
    int8_t   rssi_last;
} peer_entry_t;

// 10-byte bundle identifier used in both ESP-NOW antipackets and UART notify frames
typedef struct __attribute__((packed)) {
    uint16_t source_node;
    uint32_t creation_time;
    uint32_t sequence_number;
} antipkt_id_t;

// ESP-NOW antipacket wire format (11 bytes)
typedef struct __attribute__((packed)) {
    uint8_t      pkt_type;   // PKT_TYPE_ANTIPKT
    antipkt_id_t id;
} antipkt_pkt_t;

// Antipacket store entry
typedef struct {
    antipkt_id_t id;
    bool         is_empty;
    bool         forwarded;
    uint32_t     received_at_ms;
} stored_antipkt_t;

// BS-side delivered-bundle ring buffer , NOT packed so comparisons are always correct
typedef struct {
    uint16_t source_node;
    uint32_t creation_time;
    uint32_t sequence_number;
} bs_delivered_id_t;

typedef struct __attribute__((packed)) {
    uint16_t node_id;
    uint8_t  active_peers;
    uint8_t  store_used;
    uint8_t  store_max;
    uint8_t  channel;
} host_status_t;

typedef struct __attribute__((packed)) {
    uint16_t node_id;
    int8_t   rssi_last;
    uint8_t  active;
} host_peer_entry_t;

// Queue item for recv_cb -> rx_process_task (heap-allocated, pointer passed through queue)
#define RX_BUF_MAX 1472
typedef struct {
    uint8_t  src_mac[6];
    int8_t   rssi;
    uint16_t len;
    uint8_t  data[];   // flexible array , only actual packet bytes allocated
} rx_item_t;

/*******************************************************
 *                Globals
 *******************************************************/
static const char *TAG = "dtn_now";

static ram_bundle_t      bundle_store[MAX_BUNDLES_IN_RAM];
static stored_antipkt_t  antipkt_store[MAX_ANTIPKTS];
static bs_delivered_id_t bs_delivered[MAX_BS_DELIVERED];
static int               bs_delivered_head  = 0;
static int               bs_delivered_count = 0;
static uint32_t          local_sequence_counter = 0;
static peer_entry_t      peer_list[MAX_PEERS];
static int               peer_count = 0;
static SemaphoreHandle_t data_mutex;
static QueueHandle_t     rx_queue;
static QueueHandle_t     antipkt_notify_q;  // rx_process_task -> host_uart_task notifications

// static TX buffer , written only from bundle_tx_task (single writer, no race)
static uint8_t tx_pkt_buf[1 + sizeof(dtn_bundle_t)];

// UI state, written by rx_process_task, read by ui_update_task.
static volatile uint32_t ui_last_latency_ms = 0;
static volatile uint8_t  ui_last_hops       = 0;
static volatile int8_t   ui_last_rssi       = 0;

static bool lcd_initialized = false;   // set by lcd_init() after successful probe + init

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
    for (int i = 0; i < MAX_ANTIPKTS; i++) {
        antipkt_store[i].is_empty = true;
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

// dedup-check and store an antipacket. returns true if newly stored, false if duplicate or full.
// caller hold data_mutex.
static bool store_antipkt_locked(const antipkt_id_t *id, uint32_t ts) {
    for (int i = 0; i < MAX_ANTIPKTS; i++) {
        if (!antipkt_store[i].is_empty &&
            antipkt_store[i].id.source_node     == id->source_node &&
            antipkt_store[i].id.creation_time   == id->creation_time &&
            antipkt_store[i].id.sequence_number == id->sequence_number) {
            return false;
        }
    }
    for (int i = 0; i < MAX_ANTIPKTS; i++) {
        if (antipkt_store[i].is_empty) {
            antipkt_store[i].id             = *id;
            antipkt_store[i].is_empty       = false;
            antipkt_store[i].forwarded      = false;
            antipkt_store[i].received_at_ms = ts;
            return true;
        }
    }
    ESP_LOGW(TAG, "Antipkt store full");
    return false;
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
            peer_list[i].rssi_last    = rssi;
            peer_list[i].active = true;
            if (was_inactive) {
                // peer returned after timeout, reset forwarded so it gets everything stored
                for (int j = 0; j < MAX_BUNDLES_IN_RAM; j++) {
                    if (!bundle_store[j].is_empty) bundle_store[j].forwarded = false;
                }
                for (int j = 0; j < MAX_ANTIPKTS; j++) {
                    if (!antipkt_store[j].is_empty) antipkt_store[j].forwarded = false;
                }
                is_new = true;
            }
            xSemaphoreGive(data_mutex);
            if (is_new) {
                register_espnow_peer(mac);  // re add - was deleted on timeout
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
        peer_list[peer_count].rssi_last    = rssi;
        peer_count++;
        is_new = true;
        // new peer, so reset forwarded so they receive all stored bundles and antipackets
        for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
            if (!bundle_store[i].is_empty) bundle_store[i].forwarded = false;
        }
        for (int i = 0; i < MAX_ANTIPKTS; i++) {
            if (!antipkt_store[i].is_empty) antipkt_store[i].forwarded = false;
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

// recv callback, runs in WiFi task context , alloc exact-size item and enqueue pointer
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
    if (data_len <= 0 || data_len > RX_BUF_MAX) return;
    rx_item_t *item = malloc(sizeof(rx_item_t) + data_len);
    if (!item) {
        ESP_LOGW(TAG, "RX alloc failed, packet dropped");
        return;
    }
    memcpy(item->src_mac, info->src_addr, 6);
    item->rssi = (int8_t)info->rx_ctrl->rssi;
    item->len  = (uint16_t)data_len;
    memcpy(item->data, data, data_len);
    if (xQueueSend(rx_queue, &item, 0) != pdTRUE) {
        free(item);
        ESP_LOGW(TAG, "RX queue full, packet dropped");
    }
}

// forward declaration , defined in host_uart_task section below
static void host_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t plen);

/*******************************************************
 *                RX Processing Task
 *******************************************************/
static void rx_process_task(void *arg) {
    uint16_t my_id = get_my_node_id();
    rx_item_t *item = NULL;

    while (1) {
        // free previous item (free(NULL) is a no-op, so all continue paths are safe)
        free(item);
        item = NULL;
        if (xQueueReceive(rx_queue, &item, portMAX_DELAY) != pdTRUE) continue;
        if (item->len < 1) continue;

        uint8_t pkt_type = item->data[0];

        // beacon packet
        if (pkt_type == PKT_TYPE_BEACON && item->len >= (uint16_t)sizeof(beacon_pkt_t)) {
            beacon_pkt_t *beacon = (beacon_pkt_t *)item->data;
            add_or_refresh_peer(item->src_mac, beacon->node_id, item->rssi);

            // update peer to peer clock and
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
                printf("@NET:%u:%u:%d\n", beacon->node_id, my_id, (int)item->rssi);
            }

        // data bundle
        } else if (pkt_type == PKT_TYPE_BUNDLE) {
            size_t hdr_size = sizeof(dtn_bundle_t) - BUNDLE_PAYLOAD_SIZE;
            if ((size_t)item->len < 1 + hdr_size) continue;

            dtn_bundle_t *incoming = (dtn_bundle_t *)(item->data + 1);

            xSemaphoreTake(data_mutex, portMAX_DELAY);

            // dedup by (source_node, creation_time, sequence_number)
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
            // BS: also check bs_delivered ring buffer , catches duplicates that arrive
            // after the bundle slot was freed (common when antipackets haven't propagated yet).
            if (!already_have && my_id == BASE_STATION_NODE_ID) {
                int n = bs_delivered_count;
                for (int i = 0; i < n; i++) {
                    int idx = (bs_delivered_head - 1 - i + MAX_BS_DELIVERED) % MAX_BS_DELIVERED;
                    if (bs_delivered[idx].source_node     == incoming->source_node &&
                        bs_delivered[idx].creation_time   == incoming->creation_time &&
                        bs_delivered[idx].sequence_number == incoming->sequence_number) {
                        already_have = true;
                        break;
                    }
                }
            }

            if (already_have) {
                xSemaphoreGive(data_mutex);
                printf("@DEDUP:%u:%" PRIu32 ":%u\n",
                       incoming->source_node, incoming->sequence_number, my_id);
                continue;
            }

            // Record this delivery in the BS ring buffer so future duplicates (same source+seq)
            // are caught even after the bundle slot is freed.
            if (my_id == BASE_STATION_NODE_ID) {
                bs_delivered[bs_delivered_head].source_node     = incoming->source_node;
                bs_delivered[bs_delivered_head].creation_time   = incoming->creation_time;
                bs_delivered[bs_delivered_head].sequence_number = incoming->sequence_number;
                bs_delivered_head = (bs_delivered_head + 1) % MAX_BS_DELIVERED;
                if (bs_delivered_count < MAX_BS_DELIVERED) bs_delivered_count++;
            }

            // find empty slot
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

            // store bundle
            size_t copy_size = (size_t)item->len - 1;
            memcpy(&bundle_store[slot].bundle, incoming, copy_size);
            bundle_store[slot].is_empty  = false;
            // BS marks forwarded=true immediately so the TX task won't rebroadcast
            bundle_store[slot].forwarded = (my_id == BASE_STATION_NODE_ID);
            // take a local copy for BUNDLE_PUSH to Jetson (used after mutex release)
            dtn_bundle_t push_copy = bundle_store[slot].bundle;

            // snapshot fields needed outside mutex
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

            // rover generates DTN_RX ack telemetry bundle (inside mutex while we have space)
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

            // save payload text for BS printing (short copy, avoids using bundle_store after release)
            char payload_text[BUNDLE_PAYLOAD_SIZE];
            size_t plen = 0;
            if (my_id == BASE_STATION_NODE_ID && btelemetry) {
                plen = incoming->payload_len < (BUNDLE_PAYLOAD_SIZE - 1)
                     ? incoming->payload_len : (BUNDLE_PAYLOAD_SIZE - 1);
                memcpy(payload_text, incoming->payload, plen);
                payload_text[plen] = '\0';
            }

            xSemaphoreGive(data_mutex);

            // logging / serial output (no mutex needed)
            if (my_id == BASE_STATION_NODE_ID) {
                // adjust for clock skew: rover clocks start independently at boot.
                // src_clock_offset = rover_ms - bs_ms at last beacon, corrects creation_time
                int32_t latency = (int32_t)(now_ms() - btime) + src_clock_offset;
                if (latency < 0) latency = 0;
                printf("@METRIC:%u:%" PRIu32 ":%d:%d\n",
                       bsrc, bseq, bhops, (int)latency);
                ui_last_latency_ms = (uint32_t)latency;
                ui_last_hops       = bhops;
                ui_last_rssi       = item->rssi;
                if (btelemetry) {
                    printf("%s\n", payload_text);
                } else {
                    printf("@DTN_RX:%u:%u:%" PRIu32 ":%u:%u\n",
                           bsrc, bprev, bseq, my_id, bhops);
                }
                //free the slot (BS doesnt store bundles)
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                bundle_store[slot].is_empty = true;
                xSemaphoreGive(data_mutex);

                // broadcast antipacket so rovers clear their stores
                antipkt_pkt_t ap = {
                    .pkt_type = PKT_TYPE_ANTIPKT,
                    .id = { .source_node = bsrc, .creation_time = btime, .sequence_number = bseq },
                };
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                store_antipkt_locked(&ap.id, now_ms());
                xSemaphoreGive(data_mutex);
                esp_now_send(BROADCAST_MAC, (uint8_t *)&ap, sizeof(ap));
                ESP_LOGI(TAG, "ANTIPKT sent for src:%u seq:%" PRIu32, bsrc, bseq);
            } else {
                ESP_LOGI(TAG, "STORED src:%u seq:%" PRIu32 " hops:%u | store:%d/%d",
                         bsrc, bseq, bhops, stored_count, MAX_BUNDLES_IN_RAM);
                // push to Jetson, send only header + actual payload (not full padded struct)
                // to reduce frame size and minimise UART corruption from interleaved log output
                size_t push_size = sizeof(dtn_bundle_t) - BUNDLE_PAYLOAD_SIZE + push_copy.payload_len;
                host_send_frame(HOST_CMD_BUNDLE_PUSH, (const uint8_t *)&push_copy, (uint16_t)push_size);
            }

        // handle antipacket
        } else if (pkt_type == PKT_TYPE_ANTIPKT) {
            if (item->len < (uint16_t)sizeof(antipkt_pkt_t)) continue;
            antipkt_pkt_t *ap = (antipkt_pkt_t *)item->data;

            bool is_new = false;
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            is_new = store_antipkt_locked(&ap->id, now_ms());
            if (is_new) {
                // free any matching bundle from local store
                for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                    if (!bundle_store[i].is_empty) {
                        dtn_bundle_t *b = &bundle_store[i].bundle;
                        if (b->source_node     == ap->id.source_node &&
                            b->creation_time   == ap->id.creation_time &&
                            b->sequence_number == ap->id.sequence_number) {
                            bundle_store[i].is_empty = true;
                            ESP_LOGI(TAG, "ANTIPKT applied: freed src:%u seq:%" PRIu32,
                                     ap->id.source_node, ap->id.sequence_number);
                            break;
                        }
                    }
                }
            }
            xSemaphoreGive(data_mutex);

            if (is_new) {
                ESP_LOGI(TAG, "ANTIPKT received: src:%u seq:%" PRIu32,
                         ap->id.source_node, ap->id.sequence_number);
                // notify Jetson to drop this bundle from its store (rovers only)
                if (my_id != BASE_STATION_NODE_ID) {
                    antipkt_id_t notif = ap->id;
                    xQueueSend(antipkt_notify_q, &notif, 0);
                }
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

        // handle timed out peers
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        for (int i = 0; i < peer_count; i++) {
            if (peer_list[i].active && (ts - peer_list[i].last_seen_ms) > PEER_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Peer node=%u timed out", peer_list[i].node_id);
                peer_list[i].active = false;
                esp_now_del_peer(peer_list[i].mac);
            }
        }
        xSemaphoreGive(data_mutex);

        // BS heartbeat for visualizer
        if (my_id == BASE_STATION_NODE_ID) {
            bs_heartbeat_timer++;
            if (bs_heartbeat_timer >= 5) {
                bs_heartbeat_timer = 0;
                printf("@NET:%u:0:0\n", my_id);
            }
        }

        //generate a data bundle every 1s
        bundle_gen_timer++;
        if (bundle_gen_timer >= 1) {
            bundle_gen_timer = 0;
            if (my_id != BASE_STATION_NODE_ID) {
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                    if (bundle_store[i].is_empty) {
                        dtn_bundle_t *b = &bundle_store[i].bundle;
                        b->creation_time           = ts;
                        b->sequence_number         = local_sequence_counter++;
                        b->lifetime                = 3000000;
                        b->source_node             = my_id;
                        b->dest_node               = BASE_STATION_NODE_ID;
                        b->prev_node               = my_id;
                        b->hop_limit               = 5;
                        b->hop_count               = 0;
                        b->is_telemetry            = false;
                        b->request_delivery_report = false;
                        b->payload_len             = snprintf((char *)b->payload, BUNDLE_PAYLOAD_SIZE,
                                                              "Data from %u", my_id);
                        bundle_store[i].is_empty  = false;
                        bundle_store[i].forwarded = false;
                        ESP_LOGW(TAG, "ORIGINATED Bundle seq:%" PRIu32, b->sequence_number);
                        break;
                    }
                }
                xSemaphoreGive(data_mutex);
            }
        }

        // store dump for debugging ervy 10s
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

        // epidemic forwarding of bundles (and antipackets below) to active peers
        // snapshot active peer MACs (brief mutex hold), then send without holding mutex
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

            // hop limit
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

            // build TX packet (mutated copy, stored bundle is never modified)
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
                if (peer_ids[j] == orig_prev) continue; // dont echo back to prev hop

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

        //epidemic antipacket forwading
        // expire old antipackets first
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        for (int i = 0; i < MAX_ANTIPKTS; i++) {
            if (!antipkt_store[i].is_empty &&
                (ts - antipkt_store[i].received_at_ms) > ANTIPKT_LIFETIME_MS) {
                antipkt_store[i].is_empty = true;
            }
        }
        xSemaphoreGive(data_mutex);

        for (int i = 0; i < MAX_ANTIPKTS; i++) {
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            if (antipkt_store[i].is_empty || antipkt_store[i].forwarded || n_active == 0) {
                xSemaphoreGive(data_mutex);
                continue;
            }
            antipkt_pkt_t ap = {
                .pkt_type = PKT_TYPE_ANTIPKT,
                .id = antipkt_store[i].id,
            };
            xSemaphoreGive(data_mutex);

            bool sent_ap = false;
            for (int j = 0; j < n_active; j++) {
                esp_err_t err = esp_now_send(peer_macs[j], (uint8_t *)&ap, sizeof(ap));
                if (err == ESP_OK) sent_ap = true;
            }
            if (sent_ap) {
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                if (!antipkt_store[i].is_empty) antipkt_store[i].forwarded = true;
                xSemaphoreGive(data_mutex);
            }
        }
    }
}

/*******************************************************
 *                Host UART Interface
 *******************************************************/

// static buffers, only accessed from host_uart_task , single writer, no race
static uint8_t host_tx_buf[6 + HOST_MAX_PAYLOAD];
static uint8_t host_rx_payload[HOST_MAX_PAYLOAD];

static uint16_t crc16_update(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int j = 0; j < 8; j++)
        crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    return crc;
}

static void host_send_frame(uint8_t cmd, const uint8_t *payload, uint16_t plen) {
    if (plen > HOST_MAX_PAYLOAD) return;
    host_tx_buf[0] = HOST_SOF;
    host_tx_buf[1] = cmd;
    host_tx_buf[2] = (uint8_t)(plen & 0xFF);
    host_tx_buf[3] = (uint8_t)(plen >> 8);
    if (plen > 0 && payload) memcpy(host_tx_buf + 4, payload, plen);
    uint16_t crc = 0xFFFF;
    for (int i = 1; i < 4 + (int)plen; i++) crc = crc16_update(crc, host_tx_buf[i]);
    host_tx_buf[4 + plen]     = (uint8_t)(crc & 0xFF);
    host_tx_buf[4 + plen + 1] = (uint8_t)(crc >> 8);
    uart_write_bytes(HOST_UART_PORT, host_tx_buf, 6 + plen);
}

// returns true if a valid frame was received within timeout_ms
// payload is in host_rx_payload; cmd and plen are output params.
static bool host_recv_frame(uint8_t *cmd_out, uint16_t *plen_out, uint32_t timeout_ms) {
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000LL;
    uint8_t b;

    // scan for SOF
    do {
        if (esp_timer_get_time() > deadline) return false;
        if (uart_read_bytes(HOST_UART_PORT, &b, 1, pdMS_TO_TICKS(10)) != 1) continue;
    } while (b != HOST_SOF);

    if (uart_read_bytes(HOST_UART_PORT, cmd_out, 1, pdMS_TO_TICKS(100)) != 1) return false;

    uint8_t lb[2];
    if (uart_read_bytes(HOST_UART_PORT, lb, 2, pdMS_TO_TICKS(100)) != 2) return false;
    uint16_t plen = (uint16_t)(lb[0] | ((uint16_t)lb[1] << 8));
    if (plen > HOST_MAX_PAYLOAD) return false;
    *plen_out = plen;

    if (plen > 0 && uart_read_bytes(HOST_UART_PORT, host_rx_payload, plen, pdMS_TO_TICKS(1000)) != (int)plen)
        return false;

    uint8_t cb[2];
    if (uart_read_bytes(HOST_UART_PORT, cb, 2, pdMS_TO_TICKS(100)) != 2) return false;
    uint16_t recv_crc = (uint16_t)(cb[0] | ((uint16_t)cb[1] << 8));

    uint16_t calc_crc = 0xFFFF;
    calc_crc = crc16_update(calc_crc, *cmd_out);
    calc_crc = crc16_update(calc_crc, lb[0]);
    calc_crc = crc16_update(calc_crc, lb[1]);
    for (uint16_t i = 0; i < plen; i++) calc_crc = crc16_update(calc_crc, host_rx_payload[i]);
    return recv_crc == calc_crc;
}

// inject a raw dtn_bundle_t (plen bytes) from host into the bundle store.
// returns true on success, false if store is full or duplicate.
static bool host_inject_bundle(const uint8_t *data, uint16_t plen) {
    size_t min_hdr = sizeof(dtn_bundle_t) - BUNDLE_PAYLOAD_SIZE;
    if (plen < (uint16_t)min_hdr) return false;
    dtn_bundle_t *b = (dtn_bundle_t *)data;

    xSemaphoreTake(data_mutex, portMAX_DELAY);

    // dedup
    for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
        if (!bundle_store[i].is_empty) {
            dtn_bundle_t *s = &bundle_store[i].bundle;
            if (s->source_node    == b->source_node &&
                s->creation_time  == b->creation_time &&
                s->sequence_number == b->sequence_number) {
                xSemaphoreGive(data_mutex);
                return false; // already have it
            }
        }
    }

    for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
        if (bundle_store[i].is_empty) {
            memcpy(&bundle_store[i].bundle, b, plen);
            bundle_store[i].is_empty  = false;
            bundle_store[i].forwarded = false;
            xSemaphoreGive(data_mutex);
            ESP_LOGI(TAG, "Host: injected src=%u seq=%" PRIu32, b->source_node, b->sequence_number);
            return true;
        }
    }

    xSemaphoreGive(data_mutex);
    ESP_LOGW(TAG, "Host: store full, dropping src=%u seq=%" PRIu32, b->source_node, b->sequence_number);
    return false;
}

static void host_uart_task(void *arg) {
    uint16_t my_id = get_my_node_id();
    uint32_t last_pull_ms = 0;

    while (1) {
        uint8_t  cmd;
        uint16_t plen;
        bool got = host_recv_frame(&cmd, &plen, 100);

        // drain antipacket notifications queued by rx_process_task
        {
            antipkt_id_t notif;
            while (xQueueReceive(antipkt_notify_q, &notif, 0) == pdTRUE) {
                host_send_frame(HOST_CMD_ANTIPKT_NOTIFY, (uint8_t *)&notif, sizeof(notif));
            }
        }

        if (got) {
            switch (cmd) {
            case HOST_CMD_TX_BUNDLE: {
                bool ok = host_inject_bundle(host_rx_payload, plen);
                uint8_t status = ok ? 0 : 1;
                host_send_frame(HOST_CMD_ACK, &status, 1);
                break;
            }
            case HOST_CMD_QUERY_STATUS: {
                int used = 0, active_count = 0;
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) if (!bundle_store[i].is_empty) used++;
                for (int i = 0; i < peer_count; i++) if (peer_list[i].active) active_count++;
                xSemaphoreGive(data_mutex);
                host_status_t s = {
                    .node_id      = my_id,
                    .active_peers = (uint8_t)active_count,
                    .store_used   = (uint8_t)(used > 255 ? 255 : used),
                    .store_max    = MAX_BUNDLES_IN_RAM,
                    .channel      = ESPNOW_CHANNEL,
                };
                host_send_frame(HOST_CMD_STATUS_RESP, (uint8_t *)&s, sizeof(s));
                break;
            }
            case HOST_CMD_QUERY_PEERS: {
                host_peer_entry_t entries[MAX_PEERS];
                int n = 0;
                xSemaphoreTake(data_mutex, portMAX_DELAY);
                for (int i = 0; i < peer_count && n < MAX_PEERS; i++) {
                    entries[n].node_id   = peer_list[i].node_id;
                    entries[n].rssi_last = peer_list[i].rssi_last;
                    entries[n].active    = peer_list[i].active ? 1 : 0;
                    n++;
                }
                xSemaphoreGive(data_mutex);
                host_send_frame(HOST_CMD_PEERS_RESP, (uint8_t *)entries,
                                (uint16_t)(n * sizeof(host_peer_entry_t)));
                break;
            }
            default:
                ESP_LOGW(TAG, "Host: unknown cmd 0x%02x len=%u", cmd, plen);
                break;
            }
        }

        // periodic bundle pull from Jetson when we have peers to forward to
        uint32_t ts = now_ms();
        if (ts - last_pull_ms >= HOST_PULL_INTERVAL_MS) {
            last_pull_ms = ts;
            bool has_peers = false;
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            for (int i = 0; i < peer_count; i++) { if (peer_list[i].active) { has_peers = true; break; } }
            xSemaphoreGive(data_mutex);

            if (has_peers) {
                // drain Jetson store: pull until NO_BUNDLES or store-full/timeout
                for (int attempt = 0; attempt < MAX_BUNDLES_IN_RAM; attempt++) {
                    host_send_frame(HOST_CMD_PULL_REQ, NULL, 0);
                    uint8_t r_cmd;
                    uint16_t r_plen;
                    if (!host_recv_frame(&r_cmd, &r_plen, 500)) break;
                    if (r_cmd == HOST_CMD_NO_BUNDLES) break;
                    if (r_cmd == HOST_CMD_BUNDLE_DATA) {
                        if (!host_inject_bundle(host_rx_payload, r_plen)) break; // store full
                    }
                }
            }
        }
    }
}

static void init_host_uart(void) {
    uart_config_t cfg = {
        .baud_rate  = HOST_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(HOST_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(HOST_UART_PORT,
                                  HOST_UART_TX_PIN, HOST_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(HOST_UART_PORT, HOST_UART_RX_BUF, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "Host UART: port=%d tx=%d rx=%d baud=%d",
             HOST_UART_PORT, HOST_UART_TX_PIN, HOST_UART_RX_PIN, HOST_UART_BAUD);
}

/*******************************************************
 *                LCD driver, software I2C
 *
 * PCF8574 pin mapping: P0=RS P1=RW P2=EN P3=BL P4-P7=D4-D7
 *
 * The new ESP-IDF I2C master driver rejects all GPIOs on ESP32-C5
 * with "not usable" warnings and then NACKs data bytes.  Bit-bang
 * bypasses the driver entirely: gpio_set_level(pin,1) releases the
 * open-drain line (pull-up takes it high); gpio_set_level(pin,0)
 * drives it low.  Both SDA and SCL are configured INPUT_OUTPUT_OD.
 *******************************************************/
#define LCD_RS  0x01
#define LCD_EN  0x04
#define LCD_BL  0x08

#define BB_HALF_US  25   // ~20 kHz, conservative margin for PCF8574T

// All transitions obey I2C standard-mode timing minimums:
//   tHD_STA (START hold) >= 4.0 micro s
//   tSU_DAT (data setup before SCL high) ≥ 250 ns
//   tLOW / tHIGH >= 4.7 / 4.0 micro s

static void bb_start(void)
{
    // Ensure SDA and SCL are both high before issuing START
    gpio_set_level(LCD_SDA_PIN, 1);
    gpio_set_level(LCD_SCL_PIN, 1);
    esp_rom_delay_us(BB_HALF_US);          // bus-free / setup time
    gpio_set_level(LCD_SDA_PIN, 0);        // SDA falls while SCL high -> START
    esp_rom_delay_us(BB_HALF_US);          // tHD_STA hold (4.0 micro s min)
    gpio_set_level(LCD_SCL_PIN, 0);
    esp_rom_delay_us(BB_HALF_US);
}

static void bb_stop(void)
{
    gpio_set_level(LCD_SDA_PIN, 0);
    gpio_set_level(LCD_SCL_PIN, 1);
    esp_rom_delay_us(BB_HALF_US);          // tSU_STO setup (4.0 micro s min)
    gpio_set_level(LCD_SDA_PIN, 1);        // SDA rises while SCL high -> STOP
    esp_rom_delay_us(BB_HALF_US);          // bus-free time
}

// Returns true if worker pulled SDA low (ACK) during 9th clock
static bool bb_write_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(LCD_SDA_PIN, (data >> i) & 1);
        esp_rom_delay_us(BB_HALF_US);      // tSU_DAT data-setup before SCL↑ (250 ns min)
        gpio_set_level(LCD_SCL_PIN, 1);
        esp_rom_delay_us(BB_HALF_US);      // SCL high time (4.0 micro s min)
        gpio_set_level(LCD_SCL_PIN, 0);
        esp_rom_delay_us(BB_HALF_US);      // SCL low time (4.7 micro s min)
    }
    gpio_set_level(LCD_SDA_PIN, 1);        // release SDA for ACK bit
    esp_rom_delay_us(BB_HALF_US);
    gpio_set_level(LCD_SCL_PIN, 1);
    esp_rom_delay_us(BB_HALF_US);
    bool acked = (gpio_get_level(LCD_SDA_PIN) == 0);
    gpio_set_level(LCD_SCL_PIN, 0);
    esp_rom_delay_us(BB_HALF_US);
    return acked;
}

static esp_err_t lcd_i2c_write(const uint8_t *data, size_t len)
{
    bb_start();
    if (!bb_write_byte((LCD_I2C_ADDR << 1) | 0x00)) {
        bb_stop(); return ESP_FAIL;
    }
    for (size_t i = 0; i < len; i++) {
        if (!bb_write_byte(data[i])) { bb_stop(); return ESP_FAIL; }
    }
    bb_stop();
    return ESP_OK;
}

static void lcd_pulse_nibble(uint8_t nibble, uint8_t flags)
{
    uint8_t base = (nibble << 4) | LCD_BL | flags;
    uint8_t buf[2] = { base | LCD_EN, base };
    esp_err_t err = lcd_i2c_write(buf, 2);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD NACK nibble=0x%X flags=0x%X", nibble, flags);
    }
    esp_rom_delay_us(50);
}

static void lcd_cmd(uint8_t cmd)
{
    lcd_pulse_nibble(cmd >> 4, 0);
    lcd_pulse_nibble(cmd & 0x0F, 0);
    if (cmd < 0x04) vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_char(uint8_t c)
{
    lcd_pulse_nibble(c >> 4, LCD_RS);
    lcd_pulse_nibble(c & 0x0F, LCD_RS);
}

static void lcd_set_cursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offset[] = { 0x00, 0x40 };
    lcd_cmd(0x80 | (row_offset[row & 1] + col));
}

static void lcd_write_line(uint8_t row, const char *s)
{
    lcd_set_cursor(0, row);
    int n = 0;
    while (*s && n < LCD_COLS) { lcd_char((uint8_t)*s++); n++; }
    while (n < LCD_COLS)       { lcd_char(' ');            n++; }
}

static void lcd_init(void)
{
    // Probe: send address and check ACK before touching HD44780 init
    bb_start();
    bool acked = bb_write_byte((LCD_I2C_ADDR << 1) | 0x00);
    bb_stop();
    if (!acked) {
        ESP_LOGE(TAG, "LCD not found at 0x%02x (SDA=GPIO%d SCL=GPIO%d) , check wiring",
                 LCD_I2C_ADDR, (int)LCD_SDA_PIN, (int)LCD_SCL_PIN);
        return;
    }
    ESP_LOGI(TAG, "LCD found at 0x%02x , initializing", LCD_I2C_ADDR);

    vTaskDelay(pdMS_TO_TICKS(50));

    // HD44780 4-bit init sequence
    uint8_t buf[2];
    for (int i = 0; i < 3; i++) {
        buf[0] = 0x30 | LCD_BL | LCD_EN;
        buf[1] = 0x30 | LCD_BL;
        lcd_i2c_write(buf, 2);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    buf[0] = 0x20 | LCD_BL | LCD_EN;
    buf[1] = 0x20 | LCD_BL;
    lcd_i2c_write(buf, 2);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_cmd(0x06);
    lcd_initialized = true;
    ESP_LOGI(TAG, "LCD init complete");

    // Flash backlight once , confirms I2C is reaching PCF8574.
    // If you see the backlight (blue glow) briefly go off, I2C works,
    // adjust the contrast trim-pot until characters appear.
    uint8_t bl_off = 0x00;
    uint8_t bl_on  = LCD_BL;
    lcd_i2c_write(&bl_off, 1);
    vTaskDelay(pdMS_TO_TICKS(400));
    lcd_i2c_write(&bl_on,  1);
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_write_line(0, "DTN Base Station");
    lcd_write_line(1, "Initializing... ");
}

/*******************************************************
 *                ui_update_task  (50 Hz)
 *
 * Runs on all nodes:
 *   - LED: reflects peer connectivity + forwarding state
 * Runs on BS only (lcd_dev != NULL):
 *   - 1602 LCD: network stats, refreshed every 500 ms
 *   - Button: debounced; press switches to node-info mode for 5 s
 *******************************************************/
static void ui_update_task(void *arg)
{
    uint16_t my_id = get_my_node_id();
    bool is_bs     = (my_id == BASE_STATION_NODE_ID);

    uint32_t led_tick      = 0;
    int      btn_count     = 0;
    bool     btn_armed     = true;   // true = waiting for press, false = waiting for release
    int      lcd_cycle     = 0;
    int      alt_countdown = 0;      // LCD refresh cycles remaining in alt mode

    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / UI_TASK_HZ));
        led_tick++;

        int   n_active  = 0;
        int   n_bundles = 0;
        int8_t best_rssi = -127;
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        for (int i = 0; i < peer_count; i++) {
            if (peer_list[i].active) {
                n_active++;
                if (peer_list[i].rssi_last > best_rssi)
                    best_rssi = peer_list[i].rssi_last;
            }
        }
        for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++)
            if (!bundle_store[i].is_empty) n_bundles++;
        xSemaphoreGive(data_mutex);

        // LED
        int led_state;
        if (is_bs) {
            led_state = (n_active > 0) ? 1 : 0;
        } else {
            if (n_active > 0 && n_bundles > 0) {
                // fast blink, 4 Hz: toggle every 6 cycles at 50 Hz
                led_state = (led_tick % 12) < 6 ? 1 : 0;
            } else if (n_active > 0) {
                led_state = 1;  // solid: connected, nothing pending
            } else if (n_bundles > 0) {
                // slow blink, 1 Hz: toggle every 25 cycles at 50 Hz
                led_state = (led_tick % 50) < 25 ? 1 : 0;
            } else {
                led_state = 0;  // isolated and empty
            }
        }
        gpio_set_level(UI_LED_PIN, led_state);

        if (!is_bs) continue;   // rovers have no LCD or button

        // button debounce
        // GPIO is active-low (pull-up, button connects to GND)
        if (gpio_get_level(UI_BTN_PIN) == 0) {
            if (++btn_count >= UI_BTN_DEBOUNCE && btn_armed) {
                btn_armed     = false;
                alt_countdown = 10;
                ESP_LOGI(TAG, "Button pressed , alt mode for 5s");
            }
        } else {
            btn_count = 0;
            btn_armed = true;
        }

        // LCD refresh 
        if (++lcd_cycle < UI_LCD_EVERY) continue;
        lcd_cycle = 0;

        // 32-byte scratch buffers: snprintf writes full formatted string here
        // lcd_write_line truncates to LCD_COLS chars on output.
        char line1[32];
        char line2[32];

        if (alt_countdown > 0) {
            alt_countdown--;
            // Alt mode, node identity + bundle store fill
            snprintf(line1, sizeof(line1), "Node:%-5u Ch:%-2d", my_id, ESPNOW_CHANNEL);
            snprintf(line2, sizeof(line2), "Bndls:%-3d/%-3d", n_bundles, MAX_BUNDLES_IN_RAM);
        } else {
            // Normal mode, network health
            if (n_active > 0) {
                snprintf(line1, sizeof(line1), "Pr:%-2d  RSSI:%-4d", n_active, (int)best_rssi);
            } else {
                snprintf(line1, sizeof(line1), "Pr:0 -- ISOLATED");
            }
            uint32_t lat  = ui_last_latency_ms;
            uint8_t  hops = ui_last_hops;
            if (lat > 0) {
                snprintf(line2, sizeof(line2), "Lat:%5ums H:%u", (unsigned)(lat > 99999 ? 99999 : lat), hops);
            } else {
                snprintf(line2, sizeof(line2), "Waiting for data");
            }
        }

        if (lcd_initialized) {
            lcd_write_line(0, line1);
            lcd_write_line(1, line2);
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

    // Fixed channel , all nodes must match
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // Create queues before registering recv_cb so the callback never sees NULL handles.
    data_mutex       = xSemaphoreCreateMutex();
    rx_queue         = xQueueCreate(64, sizeof(rx_item_t *));
    antipkt_notify_q = xQueueCreate(30, sizeof(antipkt_id_t));

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

    // gpios for UI (LED + button on all nodes, LCD I2C lines on BS only)
    gpio_config_t led_cfg = {
        .pin_bit_mask = 1ULL << UI_LED_PIN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led_cfg));
    gpio_set_level(UI_LED_PIN, 0);

    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << UI_BTN_PIN,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));

    // i2c and lcd setup 
    if (my_id == BASE_STATION_NODE_ID) {
        gpio_config_t i2c_cfg = {
            .pin_bit_mask = (1ULL << LCD_SDA_PIN) | (1ULL << LCD_SCL_PIN),
            .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&i2c_cfg));
        gpio_set_level(LCD_SDA_PIN, 1);
        gpio_set_level(LCD_SCL_PIN, 1);
        lcd_init();
    }

    init_host_uart();
    init_bundle_store();

    xTaskCreate(rx_process_task, "rx_proc",    4096, NULL, 6, NULL);
    xTaskCreate(beacon_task,     "beacon",     2048, NULL, 5, NULL);
    xTaskCreate(bundle_tx_task,  "tx_loop",    4096, NULL, 4, NULL);
    xTaskCreate(host_uart_task,  "host_uart",  4096, NULL, 3, NULL);
    xTaskCreate(ui_update_task,  "ui",         2048, NULL, 2, NULL);
}
