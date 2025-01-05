# Workspace ROS 2

Workspace dalam **ROS 2** adalah seperti folder kerja utama tempat Anda menyimpan, mengorganisasi, dan membangun **package ROS 2** yang Anda kembangkan atau gunakan. Ini adalah lingkungan pengembangan yang penting untuk mengelola semua proyek ROS 2 Anda.

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
Workspace adalah fondasi dari pengembangan ROS 2. Dengan workspace yang terorganisir, Anda dapat membangun, menjalankan, dan mengelola proyek robot Anda dengan lebih efisien. Ikuti langkah-langkah di atas untuk menyiapkan dan mulai menggunakan workspace ROS 2!"
}

---

## Prasyarat

1. Pastikan ROS 2 HUMBLE sudah terinstal di sistem Anda.

---

## Langkah-Langkah Setup Workspace
### 1. Install colcon
Colcon membantu kita dalam pengelolaan package di ROS 2, seperti dalam membangun sebuah paket yang diperlukan dalam workspace.

### 2. Konfigurasi Lingkungan Pengembangan

Source setup script ROS 2:

```bash
source /opt/ros/humble/setup.bash
```
---

### 3. Buat Workspace

1. Buat direktori untuk workspace:

```bash
mkdir -p ~/ros2_ws/src
```

2. Masuk ke direktori workspace:

```bash
cd ~/ros2_ws
```

---
### 4. Build Workspace

Gunakan alat build `colcon` untuk menginisialisasi dan membangun workspace:

```bash
colcon build
```

---

### 5. Source Workspace

Setelah build selesai, source workspace untuk mengoverlay lingkungan Anda:

```bash
source install/setup.bash
```

---
### 6. Otomatisasi Setup Lingkungan

Untuk secara otomatis men-source script setup ROS 2 dan workspace Anda, tambahkan baris berikut ke file `~/.bashrc`:

```bash
source /opt/ros/<ros2_distro>/setup.bash
source ~/ros2_ws/install/setup.bash
```

Terapkan perubahan:

```bash
source ~/.bashrc
```

---

### 7. Cara Tambahkan Paket ke Workspace

Cara Untuk menambahkan pre-build paket ROS 2 ke workspace anda:

1. Masuk ke direktori `src`:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Clone repositori paket (contoh):
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

- Selalu rebuild workspace menggunakan `colcon build` setelah menambahkan atau memodifikasi paket.
- Gunakan `source install/setup.bash` pada setiap sesi terminal baru sebelum bekerja dengan workspace.

---

## Sumber Daya

Untuk tutorial lebih rinci, kunjungi [dokumentasi resmi ROS 2](https://docs.ros.org/en/rolling/index.html).
