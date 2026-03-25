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
    except ET.ParseError as e:
        print(f"Error membaca file XML: {e}")
    except FileNotFoundError:
        print(f"Error: File network '{net_file_path}' tidak ditemukan.")
    return edge_functions


net_file = "Collision.net.xml"
config_file = "Collision.sumocfg.xml"
output_folder = "output_traci"
log_filename = os.path.join(output_folder, "location_log.csv")

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
log_writer.writerow(["Waktu(s)", "ID_Agen", "Tipe_Agen", "ID_Edge", "Fungsi_Edge"])

sumoBinary = "sumo-gui"
sumoCmd = [sumoBinary, "-c", config_file, "--step-length", "0.1"]
traci.start(sumoCmd)

end_time = 600
print(f"Simulasi dimulai. Mencatat lokasi setiap agen...")

while traci.simulation.getTime() < end_time:
    traci.simulationStep()
    waktu = traci.simulation.getTime()

    vehicle_ids = traci.vehicle.getIDList()
    for veh_id in vehicle_ids:
        try:
            lane_id = traci.vehicle.getLaneID(veh_id)
            if lane_id:
                edge_id = traci.lane.getEdgeID(lane_id)
                function = edge_function_map.get(edge_id, "internal")
                log_writer.writerow([f"{waktu:.2f}", veh_id, "Vehicle", edge_id, function])
                
                
                print(f"Waktu: {waktu:.2f} | Tipe: Vehicle    | ID: {veh_id:<10} | Lokasi: {function}")

        except traci.TraCIException:
            pass

   
    person_ids = traci.person.getIDList()
    for ped_id in person_ids:
        try:
            lane_id = traci.person.getLaneID(ped_id)
            if lane_id:
                edge_id = traci.lane.getEdgeID(lane_id)
                function = edge_function_map.get(edge_id, "internal")
                log_writer.writerow([f"{waktu:.2f}", ped_id, "Pedestrian", edge_id, function])
                
                
                print(f"Waktu: {waktu:.2f} | Tipe: Pedestrian | ID: {ped_id:<10} | Lokasi: {function}")

        except traci.TraCIException:
            pass

traci.close()
log_file.close()
print(f"\nSimulasi selesai. Log lokasi telah disimpan di file: {log_filename}")
