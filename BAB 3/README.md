
# Setup Workspace ROS 2 Humble

Workspace dalam **ROS 2** adalah folder kerja utama tempat Anda menyimpan, mengorganisasi, dan membangun **package ROS 2** yang Anda kembangkan atau gunakan. Ini adalah lingkungan pengembangan yang penting untuk mengelola semua proyek ROS 2 Anda.

## Struktur Workspace 
Sebuah workspace ROS 2 biasanya memiliki struktur berikut:

```
ros2_ws/           # Folder utama workspace
├── src/           # Tempat menyimpan semua package
├── build/         # Dibuat saat Anda membangun workspace, untuk file sementara
├── install/       # Dibuat saat Anda membangun workspace, untuk file hasil build
└── log/           # File log dari proses build
```

### Penjelasan:
1. **`src/`**: Folder untuk menyimpan kode package yang Anda buat atau gunakan.
2. **`build/`**: Berisi file sementara yang dibuat secara otomatis saat membangun workspace.
3. **`install/`**: File hasil akhir setelah workspace dibangun. Digunakan agar ROS 2 mengenali package Anda.
4. **`log/`**: Log atau catatan proses yang terjadi selama build.

## Mengapa Workspace Penting?
1. **Pusat Pengembangan**: Mengorganisasi semua package proyek dalam satu lokasi.
2. **Proses Build Mudah**: `colcon build` mempermudah proses kompilasi semua package sekaligus.
3. **Interoperabilitas**: Workspace memungkinkan package ROS 2 berinteraksi dengan lancar satu sama lain.

## Kesimpulan
Workspace adalah fondasi dari pengembangan ROS 2. Dengan workspace yang terorganisir, Anda dapat membangun, menjalankan, dan mengelola proyek robot Anda dengan lebih efisien. Ikuti langkah-langkah di bawah untuk menyiapkan dan mulai menggunakan workspace ROS 2!

---

## Prasyarat

1. Pastikan **ROS 2 HUMBLE** sudah terinstal di sistem Anda.

---

## Langkah-Langkah Setup Workspace

### 1. Install colcon

Colcon adalah alat yang digunakan untuk mengelola paket di ROS 2. Instal **colcon** terlebih dahulu:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Pastikan instalasi berhasil tanpa error.

---

### 2. Konfigurasi Lingkungan Pengembangan

Setelah menginstal ROS 2, source setup script ROS 2 agar Anda bisa menggunakan perintah ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

---

### 3. Buat Workspace

Buat direktori untuk workspace dengan mengikuti langkah-langkah berikut:

1. Buat direktori workspace:

   ```bash
   mkdir -p ~/ros2_ws/src
   ```

2. Masuk ke direktori workspace:

   ```bash
   cd ~/ros2_ws
   ```

---

### 4. Build Workspace

Setelah menyiapkan workspace, gunakan `colcon` untuk membangunnya:

```bash   
colcon build
```

---

### 5. Source Workspace

Setelah proses build selesai, source workspace Anda untuk menggunakan hasil build:

```bash
source install/setup.bash
```

---

### 6. Otomatisasi Setup Lingkungan

Untuk memastikan bahwa setiap kali Anda membuka terminal baru ROS 2 dan workspace Anda langsung ter-source, jalankan command berikut:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

Terapkan perubahan dengan menjalankan perintah berikut:

```bash
source ~/.bashrc
```

---

### 7. Cara Menambahkan Paket ke Workspace (jika kamu ingin mencoba prebuild-package dan tidak harus dicoba)

Cara menambahkan paket ke workspace :

1. Masuk ke direktori `src`:

   ```bash
   cd ~/ros2_ws/src
   ```

2. Clone repositori paket yang Anda butuhkan (contoh):

   ```bash
   git clone <repository_url>
   ```

3. Kembali ke direktori workspace dan rebuild:

   ```bash
   cd ~/ros2_ws
   colcon build
   ```

---

## Catatan

- Selalu lakukan build menggunakan `colcon build` setiap kali Anda menambahkan atau mengubah paket.
- Gunakan `source install/setup.bash` untuk memastikan sesi terminal baru terhubung dengan workspace Anda.

---

## Sumber Daya

Untuk tutorial lebih rinci, kunjungi [dokumentasi resmi ROS 2](https://docs.ros.org/en/rolling/index.html).
