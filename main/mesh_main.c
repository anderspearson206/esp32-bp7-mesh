/* Mesh Internal Communication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "mesh_light.h"
#include "nvs_flash.h"

/*******************************************************
 *                Macros
 *******************************************************/

/*******************************************************
 *                Constants
 *******************************************************/
#define RX_SIZE          (1500)
#define TX_SIZE          (1460)

/*******************************************************
 *                Variable Definitions
 *******************************************************/
#define MAX_BUNDLES_IN_RAM 100
#define BUNDLE_PAYLOAD_SIZE 1024
#define BASE_STATION_NODE_ID 23768

// Tcustom bundle structure inspired by BPv7, but simplified for our mesh use case
typedef struct __attribute__((packed)) {
    uint32_t creation_time;     // Milliseconds since mesh epoch
    uint32_t sequence_number;   // Increments for absolute uniqueness
    uint32_t lifetime;          // TTL in milliseconds
    
    uint16_t source_node;       // Derived from MAC
    uint16_t dest_node;         // Target node
    uint16_t report_to_node;    // Node to send ACKs to
    uint16_t prev_node;         // last node that forwarded this bundle
    
    bool     request_delivery_report; 
    bool     is_telemetry; 
    uint8_t  hop_limit;         // max allowed hops
    uint8_t  hop_count;         // current hops
    
    size_t   payload_len;
    uint8_t  payload[BUNDLE_PAYLOAD_SIZE]; 
} dtn_bundle_t;

// local ram store
typedef struct {
    dtn_bundle_t bundle;
    bool is_empty;              
    bool forwarded;             
} ram_bundle_t;

// ram store 
static ram_bundle_t bundle_store[MAX_BUNDLES_IN_RAM];
static uint32_t local_sequence_counter = 0;

// get unique node ID from MAC address (last 2 bytes)
uint16_t get_my_node_id() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    return (mac[4] << 8) | mac[5];
}


static const char *MESH_TAG = "mesh_main";
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};
// static uint8_t tx_buf[TX_SIZE] = { 0, };
static uint8_t rx_buf[RX_SIZE] = { 0, };
static bool is_running = true;
static bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int mesh_layer = -1;
static esp_netif_t *netif_sta = NULL;

mesh_light_ctl_t light_on = {
    .cmd = MESH_CONTROL_CMD,
    .on = 1,
    .token_id = MESH_TOKEN_ID,
    .token_value = MESH_TOKEN_VALUE,
};

mesh_light_ctl_t light_off = {
    .cmd = MESH_CONTROL_CMD,
    .on = 0,
    .token_id = MESH_TOKEN_ID,
    .token_value = MESH_TOKEN_VALUE,
};

/*******************************************************
 *                Function Declarations
 *******************************************************/

 void init_bundle_store(void);

/*******************************************************
 *                Function Definitions
 *******************************************************/

void print_child_rssi() {
    wifi_sta_list_t sta_list;
    // Ask the lower-level Wi-Fi driver for the list of connected stations (children)
    esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);
    
    if (err == ESP_OK && sta_list.num > 0) {
        ESP_LOGI("RSSI_TRACKER", "--- Root Node RSSI Report (%d children) ---", sta_list.num);
        for (int i = 0; i < sta_list.num; i++) {
            ESP_LOGI("RSSI_TRACKER", "Child MAC: %02x:%02x:%02x:%02x:%02x:%02x | RSSI: %d dBm",
                     sta_list.sta[i].mac[0], sta_list.sta[i].mac[1], sta_list.sta[i].mac[2],
                     sta_list.sta[i].mac[3], sta_list.sta[i].mac[4], sta_list.sta[i].mac[5],
                     sta_list.sta[i].rssi);
        }
        ESP_LOGI("RSSI_TRACKER", "------------------------------------------");
    }
}

void esp_mesh_p2p_tx_main(void *arg)
{
    esp_err_t err;
    mesh_addr_t route_table[CONFIG_MESH_ROUTE_TABLE_SIZE];
    int route_table_size = 0;
    int bundle_generation_timer = 0;
    uint16_t my_node_id = get_my_node_id();
    int heartbeat_timer = 0;
    int flatten_timer = 0;
    int store_dump_timer = 0;

    is_running = true;

    while (is_running) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // NOTE: We intentionally do NOT gate on is_mesh_connected here.
        // For DTN ferrying we want isolated rovers AND sub-mesh roots
        // (which never receive MESH_EVENT_PARENT_CONNECTED in a rootless
        // sub-mesh) to keep generating bundles, aging TTLs, and attempting
        // forwards. esp_mesh_send() silently fails when there is no peer,
        // and that failure is handled below by reverting hop_count/prev_node.
        // The TX task itself is only spawned after the first PARENT_CONNECTED
        // (see esp_mesh_comm_p2p_start), so the mesh stack is guaranteed to
        // be initialized by the time we reach this loop.

        uint32_t current_time_ms = (uint32_t)(esp_mesh_get_tsf_time() / 1000);
        
        if (!esp_mesh_is_root()) {
            flatten_timer++;
            // every 30 seconds check to see if we can directly connect to root.
            if (flatten_timer >= 30) {
                flatten_timer = 0;
                // Layer 1 is Root. Layer 2 is directly connected to Root. 
                // Layer 3+ means we are daisy-chained.
                if (mesh_layer > 2) {
                    ESP_LOGW(MESH_TAG, "I am on Layer %d. Attempting to flatten topology...", mesh_layer);
                    // force the mesh to briefly scan to see if a better parent (like the Root) is nearby
                    esp_mesh_set_self_organized(true, true);
                }
            }
        }

        // topology log every 5 seconds
        heartbeat_timer++;
        if (heartbeat_timer >= 5) { 
            heartbeat_timer = 0;
            
            if (esp_mesh_is_root()) {
                // root node doesn't have a parent, so RSSI is 0
                printf("@NET:%u:0:0\n", my_node_id);
            } else {
                uint8_t parent_mac_5 = mesh_parent_addr.addr[5] - 1;
                uint16_t parent_id = (mesh_parent_addr.addr[4] << 8) | parent_mac_5;
                
                // query the Wi-Fi driver for parent RSSI 
                wifi_ap_record_t ap_info;
                int rssi = 0;
                if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                    rssi = ap_info.rssi;
                    
                    // try to force the mesh to reconfigure if RSSI is low, the current mesh settings don't disconnect until
                    // rssi hits -90
                    if (rssi < -87 && !esp_mesh_is_root()) {
                        ESP_LOGW(MESH_TAG, "Parent RSSI (%d dBm) hit critical threshold! Forcing proactive disconnect.", rssi);
                        esp_mesh_disconnect();
                        // The MESH_EVENT_PARENT_DISCONNECTED will handle the rest
                    }
                }
                
                // generate a telemetry bundle with the RSSI info and store it in RAM for forwarding to Root
                for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                    if (bundle_store[i].is_empty) {
                        dtn_bundle_t *b = &bundle_store[i].bundle;
                        b->creation_time = current_time_ms; 
                        b->sequence_number = local_sequence_counter++;
                        b->lifetime = 10000; 
                        b->source_node = my_node_id;
                        b->dest_node = 0;    
                        b->prev_node = my_node_id; 
                        b->hop_limit = 5;    
                        b->hop_count = 0;
                        b->is_telemetry = true; 
                        
                        // append the RSSI to the telemetry string
                        b->payload_len = snprintf((char*)b->payload, BUNDLE_PAYLOAD_SIZE, 
                                                  "@NET:%u:%u:%d", my_node_id, parent_id, rssi);
                        
                        bundle_store[i].is_empty = false;
                        bundle_store[i].forwarded = false;
                        break;
                    }
                }
            }
        }

        if (esp_mesh_is_root()) {
            print_child_rssi();
            }

        // create new data bundle every 10 seconds
        bundle_generation_timer++;
        if (bundle_generation_timer >= 10) {
            bundle_generation_timer = 0;
            for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                if (bundle_store[i].is_empty) {
                    bundle_store[i].is_empty = false;
                    bundle_store[i].forwarded = false; 
                    
                    dtn_bundle_t *b = &bundle_store[i].bundle;
                    b->creation_time = current_time_ms;
                    b->sequence_number = local_sequence_counter++;
                    b->lifetime = 300000; 
                    b->source_node = my_node_id;
                    b->dest_node = 0;    
                    b->prev_node = my_node_id; 
                    b->hop_limit = 5;    
                    b->hop_count = 0;
                    b->is_telemetry = false; 
                    b->request_delivery_report = false;
                    
                    b->payload_len = snprintf((char*)b->payload, BUNDLE_PAYLOAD_SIZE, "Data from %u", my_node_id);
                    
                    ESP_LOGW(MESH_TAG, "ORIGINATED Bundle Seq: %" PRIu32, b->sequence_number);
                    break;
                }
            }
        }

        // periodic bundle store summary
        store_dump_timer++;
        if (store_dump_timer >= 10) {
            store_dump_timer = 0;
            int total = 0;
            uint16_t seen_src[10] = {0};
            int seen_cnt[10] = {0};
            int unique = 0;
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
            if (total == 0) {
                ESP_LOGI(MESH_TAG, "@STORE: empty");
            } else {
                char dump_buf[128] = {0};
                int pos = 0;
                for (int j = 0; j < unique; j++) {
                    pos += snprintf(dump_buf + pos, sizeof(dump_buf) - pos,
                                    "%u(%d) ", seen_src[j], seen_cnt[j]);
                }
                ESP_LOGI(MESH_TAG, "@STORE: %d bundles | %s", total, dump_buf);
            }
        }

        // lifecycle management and forwarding
        esp_mesh_get_routing_table((mesh_addr_t *) &route_table,
                                   CONFIG_MESH_ROUTE_TABLE_SIZE * 6, &route_table_size);

        for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
            if (bundle_store[i].is_empty) continue;

            dtn_bundle_t *b = &bundle_store[i].bundle;

            if (b->lifetime <= 1000) {
                ESP_LOGW(MESH_TAG, "Bundle Expired (TTL). Deleting.");
                bundle_store[i].is_empty = true;
                continue;
            } else {
                b->lifetime -= 1000; // Decrement remaining life by 1 second
            }

            if (b->hop_count >= b->hop_limit) {
                ESP_LOGW(MESH_TAG, "Bundle Exceeded Hop Limit. Deleting.");
                bundle_store[i].is_empty = true;
                continue;
            }

            if (!bundle_store[i].forwarded) {
                size_t actual_tx_size = sizeof(dtn_bundle_t) - BUNDLE_PAYLOAD_SIZE + b->payload_len;

                // Make a local stack copy and mutate the copy, NOT the stored bundle.
                // Mutating the stored bundle caused hop_count to be incremented every
                // time PARENT_CONNECTED reset forwarded=false, eventually killing the
                // bundle on hop_limit even though it might never have actually reached
                // root. The store stays unchanged and each tx represents a single hop
                dtn_bundle_t tx_copy = *b;
                uint16_t original_prev = b->prev_node;

                tx_copy.prev_node = my_node_id;
                tx_copy.hop_count++;

                mesh_data_t out_data;
                out_data.data = (uint8_t *)&tx_copy;
                out_data.size = actual_tx_size;
                out_data.proto = MESH_PROTO_BIN;
                out_data.tos = MESH_TOS_P2P;

                bool sent_to_anyone = false;

                for (int j = 0; j < route_table_size; j++) {
                    uint16_t child_node_id = (route_table[j].addr[4] << 8) | route_table[j].addr[5];
                    if (child_node_id == original_prev) continue;

                    err = esp_mesh_send(&route_table[j], &out_data, MESH_DATA_P2P, NULL, 0);
                    ESP_LOGI(MESH_TAG, "TX src:%u seq:%" PRIu32 " -> peer:%u %s",
                             b->source_node, b->sequence_number, child_node_id,
                             err == ESP_OK ? "OK" : "FAIL");
                    if (err == ESP_OK) sent_to_anyone = true;
                }

                if (!esp_mesh_is_root()) {
                    uint8_t parent_mac_5 = mesh_parent_addr.addr[5] - 1;
                    uint16_t parent_node_id = (mesh_parent_addr.addr[4] << 8) | parent_mac_5;

                    if (parent_node_id != original_prev) {
                        err = esp_mesh_send(&mesh_parent_addr, &out_data, MESH_DATA_P2P, NULL, 0);
                        ESP_LOGI(MESH_TAG, "TX src:%u seq:%" PRIu32 " -> parent:%u %s",
                                 b->source_node, b->sequence_number, parent_node_id,
                                 err == ESP_OK ? "OK" : "FAIL");
                        if (err == ESP_OK) sent_to_anyone = true;
                    }
                }

                if (sent_to_anyone) {
                    bundle_store[i].forwarded = true;
                }
                // No revert needed - the stored bundle was never mutated.
            }
        }
    }
    vTaskDelete(NULL);
}


void esp_mesh_p2p_rx_main(void *arg)
{
    esp_err_t err;
    mesh_addr_t from;
    mesh_data_t data;
    int flag = 0;
    data.data = rx_buf;
    is_running = true;

    while (is_running) {
        data.size = RX_SIZE;
        err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);

        if (err != ESP_OK || !data.size) continue;

        // check if it's large enough to be a valid header
        size_t header_size = sizeof(dtn_bundle_t) - BUNDLE_PAYLOAD_SIZE;
        if (data.size >= header_size) {
            dtn_bundle_t *incoming_bundle = (dtn_bundle_t *)data.data;
            
            // check to see if unique
            bool already_have = false;
            for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                if (!bundle_store[i].is_empty) {
                    dtn_bundle_t *b = &bundle_store[i].bundle;
                    if (b->source_node == incoming_bundle->source_node &&
                        b->creation_time == incoming_bundle->creation_time &&
                        b->sequence_number == incoming_bundle->sequence_number) {
                        already_have = true;
                        break;
                    }
                }
            }

            if (already_have) {
                ESP_LOGI(MESH_TAG, "@DEDUP src:%u seq:%" PRIu32,
                         incoming_bundle->source_node, incoming_bundle->sequence_number);
            }
            // store
            if (!already_have) {
                for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
                    if (bundle_store[i].is_empty) {
                        // Store the incoming bundle
                        memcpy(&bundle_store[i].bundle, incoming_bundle, data.size);
                        bundle_store[i].is_empty = false;
                        bundle_store[i].forwarded = false; 
                        
                        uint16_t my_id = get_my_node_id();

                        // telemetry handling — check against the fixed BS node ID, not mesh role,
                        // so a ferry node that wins a sub-mesh root election still stores bundles
                        if (my_id == BASE_STATION_NODE_ID) {
                            
                            // print metric info
                            uint32_t current_time_ms = (uint32_t)(esp_mesh_get_tsf_time() / 1000);
                            uint32_t latency = current_time_ms - incoming_bundle->creation_time;

                            printf("@METRIC:%u:%" PRIu32 ":%d:%" PRIu32 "\n", 
                                incoming_bundle->source_node, 
                                incoming_bundle->sequence_number, 
                                incoming_bundle->hop_count, 
                                latency);

                            // if root, then print the telemetry directly. If it's a normal data bundle, print the RX log.
                            if (incoming_bundle->is_telemetry) {
                                printf("%.*s\n", (int)incoming_bundle->payload_len, incoming_bundle->payload);
                                bundle_store[i].forwarded = true; 
                                bundle_store[i].is_empty = true;
                            } else {
                                // root received a normal Data bundle, print the RX log
                                // Format: @DTN_RX:<source>:<prev_node>:<seq>:<receiver>:<hop_count>
                                // hop_count >= 2 indicates the bundle was relayed/ferried
                                printf("@DTN_RX:%u:%u:%" PRIu32 ":%u:%u\n",
                                        incoming_bundle->source_node,
                                        incoming_bundle->prev_node,
                                        incoming_bundle->sequence_number, my_id,
                                        incoming_bundle->hop_count);

                                // Root is the destination so drop the bundle after
                                // logging so it doesn't get re-broadcast back down
                                // the tree, which caused spurious ACK telemetry
                                // and inflated hop counts mesh-wide.
                                bundle_store[i].forwarded = true;
                                bundle_store[i].is_empty = true;
                            }
                        } else {
                            int stored_count = 0;
                            for (int k = 0; k < MAX_BUNDLES_IN_RAM; k++) {
                                if (!bundle_store[k].is_empty) stored_count++;
                            }
                            ESP_LOGI(MESH_TAG, "STORED src:%u seq:%" PRIu32 " hops:%u | store:%d/%d",
                                     incoming_bundle->source_node, incoming_bundle->sequence_number,
                                     incoming_bundle->hop_count, stored_count, MAX_BUNDLES_IN_RAM);
                            // If I am a child, and I received a normal data bundle,
                            // I must generate a new telemetry Bundle to tell the Root I got it.
                            if (!incoming_bundle->is_telemetry) {
                                for (int j = 0; j < MAX_BUNDLES_IN_RAM; j++) {
                                    if (bundle_store[j].is_empty) {
                                        dtn_bundle_t *ack = &bundle_store[j].bundle;
                                        ack->creation_time = (uint32_t)(esp_mesh_get_tsf_time() / 1000);
                                        ack->sequence_number = local_sequence_counter++;
                                        ack->lifetime = 10000;
                                        ack->source_node = my_id;
                                        ack->dest_node = 0; 
                                        ack->prev_node = my_id;
                                        ack->hop_limit = 5;
                                        ack->hop_count = 0;
                                        ack->is_telemetry = true; 
                                        
                                        // Format: @DTN_RX:<source>:<prev_node>:<seq>:<receiver>:<hop_count>
                                        ack->payload_len = snprintf((char*)ack->payload, BUNDLE_PAYLOAD_SIZE,
                                                "@DTN_RX:%u:%u:%" PRIu32 ":%u:%u",
                                                incoming_bundle->source_node, incoming_bundle->prev_node, incoming_bundle->sequence_number, my_id,
                                                incoming_bundle->hop_count);
                                        
                                        bundle_store[j].is_empty = false;
                                        bundle_store[j].forwarded = false;
                                        break;
                                    }
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

esp_err_t esp_mesh_comm_p2p_start(void)
{
    static bool is_comm_p2p_started = false;
    if (!is_comm_p2p_started) {
        is_comm_p2p_started = true;
        xTaskCreate(esp_mesh_p2p_tx_main, "MPTX", 3072, NULL, 5, NULL);
        xTaskCreate(esp_mesh_p2p_rx_main, "MPRX", 3072, NULL, 5, NULL);
    }
    return ESP_OK;
}

void mesh_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint16_t last_layer = 0;

    switch (event_id) {
    case MESH_EVENT_STARTED: {
        esp_mesh_get_id(&id);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_MESH_STARTED>ID:"MACSTR"", MAC2STR(id.addr));
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
        is_mesh_connected = false;
        mesh_layer = esp_mesh_get_layer();
    }
    break;
    case MESH_EVENT_CHILD_CONNECTED: {
        mesh_event_child_connected_t *child_connected = (mesh_event_child_connected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_CONNECTED>aid:%d, "MACSTR"",
                 child_connected->aid,
                 MAC2STR(child_connected->mac));
    }
    break;
    case MESH_EVENT_CHILD_DISCONNECTED: {
        mesh_event_child_disconnected_t *child_disconnected = (mesh_event_child_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHILD_DISCONNECTED>aid:%d, "MACSTR"",
                 child_disconnected->aid,
                 MAC2STR(child_disconnected->mac));
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_ADD: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_ADD>add %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_ROUTING_TABLE_REMOVE: {
        mesh_event_routing_table_change_t *routing_table = (mesh_event_routing_table_change_t *)event_data;
        ESP_LOGW(MESH_TAG, "<MESH_EVENT_ROUTING_TABLE_REMOVE>remove %d, new:%d, layer:%d",
                 routing_table->rt_size_change,
                 routing_table->rt_size_new, mesh_layer);
    }
    break;
    case MESH_EVENT_NO_PARENT_FOUND: {
        mesh_event_no_parent_found_t *no_parent = (mesh_event_no_parent_found_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NO_PARENT_FOUND>scan times:%d",
                 no_parent->scan_times);
    }

    break;
    case MESH_EVENT_PARENT_CONNECTED: {
        uint16_t my_id = get_my_node_id();
        uint16_t parent_id = (mesh_parent_addr.addr[4] << 8) | mesh_parent_addr.addr[5];
        // emit strict topology log
        printf("@NET:%u:%u\n", my_id, parent_id);
        mesh_event_connected_t *connected = (mesh_event_connected_t *)event_data;
        esp_mesh_get_id(&id);
        mesh_layer = connected->self_layer;
        memcpy(&mesh_parent_addr.addr, connected->connected.bssid, 6);
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_CONNECTED>layer:%d-->%d, parent:"MACSTR"%s, ID:"MACSTR", duty:%d",
                 last_layer, mesh_layer, MAC2STR(mesh_parent_addr.addr),
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "", MAC2STR(id.addr), connected->duty);
        last_layer = mesh_layer;
        mesh_connected_indicator(mesh_layer);
        is_mesh_connected = true;
        // init_bundle_store();
        for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
            if (!bundle_store[i].is_empty) {
                bundle_store[i].forwarded = false;
            }
        }
        if (esp_mesh_is_root()) {
            esp_netif_dhcpc_stop(netif_sta);
            esp_netif_dhcpc_start(netif_sta);
        }
        esp_mesh_comm_p2p_start();
    }
    break;
    case MESH_EVENT_PARENT_DISCONNECTED: {
        mesh_event_disconnected_t *disconnected = (mesh_event_disconnected_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_PARENT_DISCONNECTED>reason:%d",
                 disconnected->reason);
        is_mesh_connected = false;
        mesh_disconnected_indicator();
        mesh_layer = esp_mesh_get_layer();

        if (!esp_mesh_is_root()) {
            ESP_LOGW(MESH_TAG, "Link lost! Forcing aggressive re-election of a new parent...");
            // true, true = Enable self-organization AND force an immediate re-scan for the best parent
            esp_mesh_set_self_organized(true, true);
        }
    }
    break;
    case MESH_EVENT_LAYER_CHANGE: {
        mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
        mesh_layer = layer_change->new_layer;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_LAYER_CHANGE>layer:%d-->%d%s",
                 last_layer, mesh_layer,
                 esp_mesh_is_root() ? "<ROOT>" :
                 (mesh_layer == 2) ? "<layer2>" : "");
        last_layer = mesh_layer;
        mesh_connected_indicator(mesh_layer);
    }
    break;
    case MESH_EVENT_ROOT_ADDRESS: {
        mesh_event_root_address_t *root_addr = (mesh_event_root_address_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_ADDRESS>root address:"MACSTR"",
                 MAC2STR(root_addr->addr));
    }
    break;
    case MESH_EVENT_VOTE_STARTED: {
        mesh_event_vote_started_t *vote_started = (mesh_event_vote_started_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_VOTE_STARTED>attempts:%d, reason:%d, rc_addr:"MACSTR"",
                 vote_started->attempts,
                 vote_started->reason,
                 MAC2STR(vote_started->rc_addr.addr));
    }
    break;
    case MESH_EVENT_VOTE_STOPPED: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_VOTE_STOPPED>");
        break;
    }
    case MESH_EVENT_ROOT_SWITCH_REQ: {
        mesh_event_root_switch_req_t *switch_req = (mesh_event_root_switch_req_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_SWITCH_REQ>reason:%d, rc_addr:"MACSTR"",
                 switch_req->reason,
                 MAC2STR( switch_req->rc_addr.addr));
    }
    break;
    case MESH_EVENT_ROOT_SWITCH_ACK: {
        /* new root */
        mesh_layer = esp_mesh_get_layer();
        esp_mesh_get_parent_bssid(&mesh_parent_addr);
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_SWITCH_ACK>layer:%d, parent:"MACSTR"", mesh_layer, MAC2STR(mesh_parent_addr.addr));
    }
    break;
    case MESH_EVENT_TODS_STATE: {
        mesh_event_toDS_state_t *toDs_state = (mesh_event_toDS_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_TODS_REACHABLE>state:%d", *toDs_state);
    }
    break;
    case MESH_EVENT_ROOT_FIXED: {
        mesh_event_root_fixed_t *root_fixed = (mesh_event_root_fixed_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROOT_FIXED>%s",
                 root_fixed->is_fixed ? "fixed" : "not fixed");
    }
    break;
    case MESH_EVENT_ROOT_ASKED_YIELD: {
        mesh_event_root_conflict_t *root_conflict = (mesh_event_root_conflict_t *)event_data;
        ESP_LOGI(MESH_TAG,
                 "<MESH_EVENT_ROOT_ASKED_YIELD>"MACSTR", rssi:%d, capacity:%d",
                 MAC2STR(root_conflict->addr),
                 root_conflict->rssi,
                 root_conflict->capacity);
    }
    break;
    case MESH_EVENT_CHANNEL_SWITCH: {
        mesh_event_channel_switch_t *channel_switch = (mesh_event_channel_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_CHANNEL_SWITCH>new channel:%d", channel_switch->channel);
    }
    break;
    case MESH_EVENT_SCAN_DONE: {
        mesh_event_scan_done_t *scan_done = (mesh_event_scan_done_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_SCAN_DONE>number:%d",
                 scan_done->number);
    }
    break;
    case MESH_EVENT_NETWORK_STATE: {
        mesh_event_network_state_t *network_state = (mesh_event_network_state_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_NETWORK_STATE>is_rootless:%d",
                 network_state->is_rootless);
    }
    break;
    case MESH_EVENT_STOP_RECONNECTION: {
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOP_RECONNECTION>");
    }
    break;
    case MESH_EVENT_FIND_NETWORK: {
        mesh_event_find_network_t *find_network = (mesh_event_find_network_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_FIND_NETWORK>new channel:%d, router BSSID:"MACSTR"",
                 find_network->channel, MAC2STR(find_network->router_bssid));
    }
    break;
    case MESH_EVENT_ROUTER_SWITCH: {
        mesh_event_router_switch_t *router_switch = (mesh_event_router_switch_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_ROUTER_SWITCH>new router:%s, channel:%d, "MACSTR"",
                 router_switch->ssid, router_switch->channel, MAC2STR(router_switch->bssid));
        if (esp_mesh_is_root()) {
            is_mesh_connected = true;
            esp_mesh_comm_p2p_start();
        }
    }
    break;
    case MESH_EVENT_PS_PARENT_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_PARENT_DUTY>duty:%d", ps_duty->duty);
    }
    break;
    case MESH_EVENT_PS_CHILD_DUTY: {
        mesh_event_ps_duty_t *ps_duty = (mesh_event_ps_duty_t *)event_data;
        ESP_LOGI(MESH_TAG, "<MESH_EVENT_PS_CHILD_DUTY>cidx:%d, "MACSTR", duty:%d", ps_duty->child_connected.aid-1,
                MAC2STR(ps_duty->child_connected.mac), ps_duty->duty);
    }
    break;
    default:
        ESP_LOGI(MESH_TAG, "unknown id:%" PRId32 "", event_id);
        break;
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(MESH_TAG, "<IP_EVENT_STA_GOT_IP>IP:" IPSTR, IP2STR(&event->ip_info.ip));

}

void init_bundle_store(void) {
    for (int i = 0; i < MAX_BUNDLES_IN_RAM; i++) {
        bundle_store[i].is_empty = true;
        bundle_store[i].forwarded = false;
        
        // Zero out the entire inner BPv7 bundle struct securely
        memset(&bundle_store[i].bundle, 0, sizeof(dtn_bundle_t));
    }
    ESP_LOGI(MESH_TAG, "Bundle RAM Store Initialized.");
}

void app_main(void)
{
    ESP_ERROR_CHECK(mesh_light_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    /*  tcpip initialization */
    ESP_ERROR_CHECK(esp_netif_init());
    /*  event initialization */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /*  create network interfaces for mesh (only station instance saved for further manipulation, soft AP instance ignored */
    ESP_ERROR_CHECK(esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));
    /*  wifi initialization */
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&config));
    
    wifi_country_t country = {
        .cc = "US",
        .schan = 1,
        .nchan = 11,
        .policy = WIFI_COUNTRY_POLICY_AUTO
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_start());
    /*  mesh initialization */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));
    /*  set mesh topology */
    ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
    /*  set mesh max layer according to the topology */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
#ifdef CONFIG_MESH_ENABLE_PS
    /* Enable mesh PS function */
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    /* better to increase the associate expired time, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));
    /* better to increase the announce interval to avoid too much management traffic, if a small duty cycle is set. */
    ESP_ERROR_CHECK(esp_mesh_set_announce_interval(600, 3300));
#else
    // Disable mesh PS function 
    ESP_ERROR_CHECK(esp_mesh_disable_ps());
    
    // drop a dead parent after 10 seconds instead of 60s
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));
    
    // fast Discovery: Broadcast presence every 200-500ms so approaching rovers link faster
    // ESP_ERROR_CHECK(esp_mesh_set_announce_interval(500,1000));
    
#endif
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    /* mesh ID */
    memcpy((uint8_t *) &cfg.mesh_id, MESH_ID, 6);
    /* router — use a fake BSSID with ssid_len=0 so the mesh operates standalone.
     * ssid_len=0 alone causes esp_mesh_set_config to fail (requires non-zero BSSID).
     * With BSSID-only config the STA only passively scans for that specific MAC
     * and never initiates probe/auth — so no auth-timeout disconnects (reason:201).
     * With a fake SSID the STA actively probes, times out every ~11s, resets, and
     * causes all children to briefly disconnect in a loop.
     * The BS wins root via MESH_ROOT type. Fixed channel required. */
    cfg.channel = 6;
    cfg.router.ssid_len = 0;
    uint8_t fake_bssid[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    memcpy(cfg.router.bssid, fake_bssid, sizeof(fake_bssid));
    /* mesh softAP */
    ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *) &cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));
    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    // /* --- PROACTIVE ROVER HANDOFF CONFIGURATION --- */
    
    // // 1. Shift the global RSSI tier definitions
    // mesh_rssi_threshold_t rssi_thresh;
    // esp_mesh_get_rssi_threshold(&rssi_thresh);
    // // Make the mesh highly sensitive to signal degradation by shifting thresholds up
    // rssi_thresh.high = -70;
    // rssi_thresh.medium = -75;
    // rssi_thresh.low = -80; 
    // ESP_ERROR_CHECK(esp_mesh_set_rssi_threshold(&rssi_thresh));

    // // 2. Configure the aggressive parent-switching logic
    // mesh_switch_parent_t switch_paras;
    // esp_mesh_get_switch_parent_paras(&switch_paras);
    
    // // Default duration is 60000ms. Lower to 3 seconds for fast-moving rovers
    // switch_paras.duration_ms = 3000;   
    
    // // If the current parent drops below -80 dBm, start the 3-second timer
    // switch_paras.cnx_rssi = -80;       
    
    // // The new candidate parent must have at least a -70 dBm signal to be selected
    // switch_paras.select_rssi = -70;    
    
    // // Disassociate with the current parent and switch when the new candidate is > -70 dBm
    // switch_paras.switch_rssi = -70;    
    
    // // Minimum RSSI required to fall back and connect directly to the Root node
    // switch_paras.backoff_rssi = -75;   
    
    // ESP_ERROR_CHECK(esp_mesh_set_switch_parent_paras(&switch_paras));
    // /* --------------------------------------------- */

    uint16_t my_id = get_my_node_id();
    
    // Replace 23768 with the actual ID of your Base Station (COM5)
    // Clear any stale fix_root=true persisted in NVS from previous firmware.
    // Must be called on all nodes before esp_mesh_start().
    ESP_ERROR_CHECK(esp_mesh_fix_root(false));
    if (my_id == BASE_STATION_NODE_ID) {
        ESP_LOGI(MESH_TAG, "I am the Base Station. Forcing ROOT role.");
        ESP_ERROR_CHECK(esp_mesh_set_type(MESH_ROOT));
    } else {
        ESP_LOGI(MESH_TAG, "I am a Rover. Starting as IDLE role.");
        ESP_ERROR_CHECK(esp_mesh_set_type(MESH_IDLE));
    }
    init_bundle_store();
    /* mesh start */
    ESP_ERROR_CHECK(esp_mesh_start());
#ifdef CONFIG_MESH_ENABLE_PS
    /* set the device active duty cycle. (default:10, MESH_PS_DEVICE_DUTY_REQUEST) */
    ESP_ERROR_CHECK(esp_mesh_set_active_duty_cycle(CONFIG_MESH_PS_DEV_DUTY, CONFIG_MESH_PS_DEV_DUTY_TYPE));
    /* set the network active duty cycle. (default:10, -1, MESH_PS_NETWORK_DUTY_APPLIED_ENTIRE) */
    ESP_ERROR_CHECK(esp_mesh_set_network_duty_cycle(CONFIG_MESH_PS_NWK_DUTY, CONFIG_MESH_PS_NWK_DUTY_DURATION, CONFIG_MESH_PS_NWK_DUTY_RULE));
#endif
    ESP_LOGI(MESH_TAG, "mesh starts successfully, heap:%" PRId32 ", %s<%d>%s, ps:%d",  esp_get_minimum_free_heap_size(),
             esp_mesh_is_root_fixed() ? "root fixed" : "root not fixed",
             esp_mesh_get_topology(), esp_mesh_get_topology() ? "(chain)":"(tree)", esp_mesh_is_ps_enabled());
}
