# Installasi ROS 2 Humble di Ubuntu

Panduan ini menjelaskan langkah-langkah untuk menginstal ROS 2 Humble Hawksbill di Ubuntu dengan menggunakan paket debian (Ubuntu 22.04 direkomendasikan).

## Persyaratan Sebelum Menginstal ROS 2

Sebelum melakukan instalasi ROS 2, pastikan Anda memenuhi beberapa persyaratan berikut:

### 2.1 Kebutuhan Sistem

#### Sistem Operasi
- **Ubuntu 22.04 (Jammy Jellyfish)** adalah OS yang secara resmi didukung.
- Sistem operasi lain bisa digunakan, tetapi tidak dijamin kompatibilitasnya.

#### Perangkat Keras (Hardware)

##### Prosesor:
- **Minimum**: Prosesor dual-core 64-bit.
- **Rekomendasi**: Prosesor quad-core atau lebih tinggi untuk simulasi yang kompleks.

##### RAM (Memori):
- **Minimum**: 4 GB.
- **Rekomendasi**: 8 GB atau lebih, terutama jika menggunakan banyak node atau simulasi intensif.

##### Penyimpanan (Storage):
- **Minimum**: 10 GB ruang kosong untuk instalasi.
- **Rekomendasi**: Lebih dari 20 GB jika menggunakan proyek besar atau simulasi.

##### Kartu Grafis:
- **Diperlukan**: Dukungan OpenGL untuk alat visualisasi seperti Gazebo dan RViz, atau PyBullet.
- **Rekomendasi**: GPU dedicated seperti NVIDIA jika menggunakan simulasi visual yang berat.

#### Perangkat Lunak

- **CMake**: Versi 3.16 atau lebih baru.
- **Python**: Versi 3.11 atau kompatibel.
- **Colcon**: Untuk membangun workspace ROS 2.
- **Compiler**: `gcc`, `g++` (GNU Compiler Collection), dan `make`.
- **Git**: Untuk manajemen kode sumber.
- **Curl/Wget**: Untuk mengambil skrip instalasi.

#### Rekomendasi Visual Studio Code

##### Unduh dan Instal VS Code:
- Kunjungi [Visual Studio Code](https://code.visualstudio.com/) untuk mengunduh.
- Pastikan sesuai dengan arsitektur sistem Anda (x64).

##### Ekstensi Wajib untuk ROS 2 di VS Code:
- **ROS Extension Pack**: Instal melalui VS Code Extensions Marketplace. Paket ini mencakup:
  - **ROS**
  - **Python**
  - **C/C++**
- **Colcon Helper**: Mempermudah proses build workspace ROS 2.

### Catatan
Jika ada beberapa syarat yang tidak bisa dipenuhi, disarankan untuk tidak melakukan instalasi ROS 2.

---

## Langkah-Langkah Instalasi

### 1. Persiapan Sistem

1. Update paket pada sistem Anda:

   ```bash
   sudo apt update && sudo apt upgrade
   ```

2. Install tools umum yang diperlukan:

   ```bash
   sudo apt install -y curl gnupg lsb-release
   ```

---

### 2. Tambahkan Repositori ROS 2

1. Tambahkan GPG key:

   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

2. Tambahkan ROS 2 repository ke sistem Anda:

   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. Update cache paket:

   ```bash
   sudo apt update
   ```

---

### 3. Instalasi ROS 2 Humble

1. Install desktop full ROS 2:

   ```bash
   sudo apt install -y ros-humble-desktop
   ```

   Jika Anda memerlukan versi minimal:

   ```bash
   sudo apt install -y ros-humble-ros-base
   ```

2. Sumberkan setup ROS secara otomatis saat terminal dibuka:

   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

---

### 4. Instalasi Dependencies Tambahan (Opsional)

Install colcon dan alat lainnya jika Anda berencana untuk membangun paket dari source:

```bash
sudo apt install -y python3-colcon-common-extensions python3-argcomplete
```

---

### 5. Verifikasi Instalasi

Cek instalasi ROS 2 dengan menjalankan perintah berikut:

```bash
ros2 --version
```

---

## Sumber

Referensi resmi: [Dokumentasi Instalasi ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

---

Jika Anda mengalami masalah selama instalasi, pastikan untuk memeriksa kembali setiap langkah dan referensi dokumentasi resmi. Selamat mencoba!

