import xml.etree.ElementTree as ET
import traci
import csv
import os
import sys


def load_edge_functions(net_file_path):

    edge_functions = {}
    try:
        tree = ET.parse(net_file_path)
        root = tree.getroot()
        for edge in root.findall('edge'):
            edge_id = edge.get('id')
            function = edge.get('function', 'normal')
            if edge_id:
                edge_functions[edge_id] = function
    except (ET.ParseError, FileNotFoundError) as e:
        print(f"Error membaca file network: {e}")
    return edge_functions

def calculate_distance(pos1, pos2):
    import math
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

distance_threshold = 2.0
speed_threshold_kmh = 20.0
speed_threshold_ms = speed_threshold_kmh / 3.6

net_file = "Collision.net.xml"
config_file = "Collision.sumocfg.xml"
output_folder = "output_traci"
log_filename = os.path.join(output_folder, "potensi_tabrakan.csv")

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Harap deklarasikan environment variable 'SUMO_HOME'")


print(f"Membaca data fungsi edge dari {net_file}...")
edge_function_map = load_edge_functions(net_file)
if not edge_function_map:
    print("Gagal memuat data edge, program berhenti.")
    sys.exit()
print("Data edge berhasil dimuat.")

os.makedirs(output_folder, exist_ok=True)
log_file = open(log_filename, "w", newline='')
log_writer = csv.writer(log_file)

log_writer.writerow(["Waktu(s)", "ID Kendaraan", "ID Pejalan Kaki", "Jarak(m)", "Kecepatan(m/s)"])
already_warned = set()

sumoBinary = "sumo"
sumoCmd = [sumoBinary, "-c", config_file, "--step-length", "0.1"]
traci.start(sumoCmd)

end_time = 600
print(f"Simulasi dimulai. Mengabaikan pejalan kaki di edge 'normal'...")

while traci.simulation.getTime() < end_time:
    traci.simulationStep()
    vehicle_ids = traci.vehicle.getIDList()
    person_ids = traci.person.getIDList()

    for veh_id in vehicle_ids:
        try:
            for ped_id in person_ids:
                ped_lane_id = traci.person.getLaneID(ped_id)
                if not ped_lane_id: continue

                ped_edge_id = traci.lane.getEdgeID(ped_lane_id)
                edge_function = edge_function_map.get(ped_edge_id, "internal")

                if edge_function == 'normal':
                    continue

                veh_pos = traci.vehicle.getPosition(veh_id)
                veh_speed = traci.vehicle.getSpeed(veh_id)
                ped_pos = traci.person.getPosition(ped_id)
                distance = calculate_distance(veh_pos, ped_pos)

                if distance < distance_threshold and veh_speed > speed_threshold_ms:
                    key = (veh_id, ped_id)
                    if key not in already_warned:
                        already_warned.add(key)
                        waktu = traci.simulation.getTime()
                        print(f"[PERINGATAN] Potensi tabrakan antara kendaraan {veh_id} dan pejalan kaki {ped_id} pada waktu {waktu:.2f}s!")
                        print(f"    → Jarak: {distance:.2f} m, Kecepatan: {veh_speed * 3.6:.2f} km/j")
                        
                        log_writer.writerow([waktu, veh_id, ped_id, f"{distance:.2f}", f"{veh_speed:.2f}"])
        
        except traci.TraCIException:
            pass

traci.close()
log_file.close()
print(f"\nSimulasi selesai. Data potensi tabrakan telah disimpan di file: {log_filename}")
