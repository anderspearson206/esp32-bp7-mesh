import serial
import threading
import time
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque, defaultdict

SERIAL_PORT = 'COM5'  # Change to /dev/ttyUSB0 for Linux/Mac
BAUD_RATE = 115200
TIMEOUT_SECONDS = 15 


G = nx.DiGraph()
recent_transfers = [] #
lock = threading.Lock()
node_latencies = defaultdict(lambda: deque(maxlen=20))

def serial_reader_thread(port):
    """Reads serial data from a specific port and updates the network graph state."""
    while True: 
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=1)
            print(f"Connected to {port}. Listening for DTN telemetry...")
            
            while True:
                # Read raw bytes first to avoid immediate decode crashes
                raw_line = ser.readline()
                if not raw_line:
                    continue
                
                try:
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                except Exception:
                    continue

                if not line:
                    continue
                    

                # parse topology updates
                if line.startswith("@NET:"):
                    print(f"Received: {line}")
                    parts = line.split(':')
                    if len(parts) >= 4:
                        node_id, parent, rssi = parts[1], parts[2], parts[3] # rename to node_id
                        with lock:
                            current_time = time.time()
                            edges_to_remove = [(u, v) for u, v in G.edges() if u == node_id]
                            G.remove_edges_from(edges_to_remove)
                            
                            # update tsamp for this node (or add it if new)
                            G.add_node(node_id, last_seen=current_time)
                            
                            if parent != "0": 
                                G.add_node(parent) 
                                G.add_edge(node_id, parent, link_type='mesh', rssi=rssi)
                                
                # movement bundles
                elif line.startswith("@DTN_RX:"):
                    print(f"Received: {line}")
                    parts = line.split(':')
                    if len(parts) >= 4:
                        source, seq, receiver = parts[1], parts[2], parts[3]
                        with lock:
                            current_time = time.time()
                            G.add_node(source, last_seen=current_time)
                            G.add_node(receiver, last_seen=current_time)

                            recent_transfers.append({
                                'src': source,
                                'dst': receiver,
                                'time': current_time
                            })
                
                # parse latency metrics
                elif line.startswith("@METRIC:"):
                    parts = line.split(':')
                    # Format: @METRIC:<source_node>:<seq_num>:<hop_count>:<latency>
                    if len(parts) >= 5:
                        source_node = parts[1]
                        try:
                            latency_ms = int(parts[4])
                            
                            # filter out unrealistic latencies (e.g., negative or excessively high values)
                            if 0 <= latency_ms < 300000: 
                                with lock:
                                    node_latencies[source_node].append(latency_ms)
                        except ValueError:
                            pass

        except (serial.SerialException, serial.PortNotOpenError) as e:
            print(f"Connection lost on {port}: {e}. Retrying in 2 seconds...")
            time.sleep(2)
        except Exception as e:
            print(f"Unexpected error on {port}: {e}")
            time.sleep(2)
            
            
def update_graph(frame):
    plt.clf() 
    with lock:
        current_time = time.time()
        
        nodes_to_remove = [
            n for n, attr in G.nodes(data=True) 
            if current_time - attr.get('last_seen', 0) > TIMEOUT_SECONDS
        ]
        
        for n in nodes_to_remove:
            print(f"Node {n} timed out. Removing from visualizer.")
            G.remove_node(n)

        if len(G.nodes) == 0:
            plt.title("Waiting for Mesh Data...")
            return
        active_transfers = [t for t in recent_transfers if current_time - t['time'] < 1.0]
        recent_transfers[:] = active_transfers 

        try:
            pos = nx.nx_agraph.graphviz_layout(G, prog="dot")
        except ImportError:
            pos = nx.spring_layout(G, seed=42)

        nx.draw_networkx_nodes(G, pos, node_size=700, node_color='lightblue')
        nx.draw_networkx_labels(G, pos, font_size=12, font_weight="bold")
        nx.draw_networkx_edges(G, pos, edge_color='gray', arrows=True)

        edge_labels = nx.get_edge_attributes(G, 'rssi')
        formatted_labels = {k: f"{v} dBm" for k, v in edge_labels.items()}
        nx.draw_networkx_edge_labels(G, pos, edge_labels=formatted_labels, font_size=10, font_color='blue')

        for transfer in active_transfers:
            src = transfer['src']
            dst = transfer['dst']
            if src in G and dst in G:
                nx.draw_networkx_edges(G, pos, edgelist=[(src, dst)], 
                                       edge_color='red', width=3.0, arrows=True)
                
        # Calculate per-node averages
        latency_strings = []
        for node, latencies in node_latencies.items():
            # Only display if we have data and the node is currently active in the graph
            if len(latencies) > 0 and node in G.nodes:
                avg_lat = sum(latencies) / len(latencies)
                latency_strings.append(f"Rover {node}: {avg_lat:.1f} ms")

        # Build the display string
        if latency_strings:
            # Join the stats together, wrapping to a new line every 3 rovers to keep it clean
            formatted_stats = "\n".join(
                [" | ".join(latency_strings[i:i+3]) for i in range(0, len(latency_strings), 3)]
            )
            title_str = f"Live DTN Rover Mesh\nAverage Latencies:\n{formatted_stats}"
        else:
            title_str = "Live DTN Rover Mesh\nWaiting for metric data..."

        plt.title(title_str, fontsize=12, fontweight='bold', pad=15)
        plt.axis('off')

if __name__ == '__main__':
    thread = threading.Thread(target=serial_reader_thread, args=(SERIAL_PORT,), daemon=True)
    
    try:
        thread.start()

        fig = plt.figure(figsize=(8, 6))
        ani = animation.FuncAnimation(fig, update_graph, interval=500, cache_frame_data=False)
        
        plt.show()
    except KeyboardInterrupt:
        print("\nShutting down visualizer...")