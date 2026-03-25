import pandas as pd
import matplotlib.pyplot as plt

# --- KONFIGURASI ---
# Pastikan nama file ini sesuai dengan file log Anda
LOG_FILE = "results/Hasil Cam Logger.csv"

def plot_trajectories(log_file_path):
    """
    Fungsi untuk membaca log CAM dan mem-plot trajektori pengirim.
    """
    print(f"Membaca file log dari: {log_file_path}")
    try:
        # Membaca file CSV dengan pemisah (separator) titik koma (;)
        df = pd.read_csv(log_file_path, sep=';')
    except FileNotFoundError:
        print(f"Error: File '{log_file_path}' tidak ditemukan.")
        return

    print("Data berhasil dimuat. Memproses trajektori...")

    # Kita hanya butuh satu set data posisi unik untuk setiap pengirim pada setiap waktu
    # Ini untuk membersihkan data jika ada satu pesan yang diterima oleh banyak penerima
    df_unique_points = df.drop_duplicates(subset=['Time(s)', 'Sender_Node'])

    # Pisahkan data untuk mobil dan pejalan kaki berdasarkan tipe pengirim
    df_car = df_unique_points[df_unique_points['Sender_Type'] == 'Car'].sort_values('Time(s)')
    df_person = df_unique_points[df_unique_points['Sender_Type'] == 'Person'].sort_values('Time(s)')

    # --- MEMBUAT PLOT VISUAL ---
    plt.figure(figsize=(12, 10))
    
    # Plot trajektori mobil (misalnya, warna merah dengan penanda titik)
    plt.plot(df_car['Sender_Longitude'], df_car['Sender_Latitude'], color='red', linestyle='-', marker='.', markersize=4, label='Trajektori Kendaraan (Car)')
    
    # Plot trajektori pejalan kaki (misalnya, warna biru)
    plt.plot(df_person['Sender_Longitude'], df_person['Sender_Latitude'], color='blue', linestyle='-', marker='.', markersize=4, label='Trajektori Pejalan Kaki (Person)')

    # Menandai titik awal dan akhir untuk kejelasan
    if not df_car.empty:
        plt.plot(df_car.iloc[0]['Sender_Longitude'], df_car.iloc[0]['Sender_Latitude'], 'go', markersize=8, label='Start Mobil') # Titik awal mobil (hijau)
    if not df_person.empty:
        plt.plot(df_person.iloc[0]['Sender_Longitude'], df_person.iloc[0]['Sender_Latitude'], 'yo', markersize=8, label='Start Pejalan Kaki') # Titik awal pejalan kaki (kuning)

    # Pengaturan Grafik
    plt.title('Rekonstruksi Trajektori Kendaraan dan Pejalan Kaki')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    plt.axis('equal') # Sangat penting agar skala peta tidak terdistorsi
    plt.show()

# Panggil fungsi utama untuk menjalankan analisis
if __name__ == "__main__":
    plot_trajectories(LOG_FILE)
