# Setup Workspace ROS 2

Panduan ini menjelaskan langkah-langkah untuk menyiapkan workspace ROS 2 untuk pengembangan. Ikuti langkah-langkah berikut untuk mempersiapkan lingkungan dan membuat workspace untuk paket ROS 2.


---

## Prasyarat

1. Pastikan ROS 2 HUMBLE sudah terinstal di sistem Anda.

---

## Langkah-Langkah Setup Workspace

### 1. Konfigurasi Lingkungan Pengembangan

Source setup script ROS 2:

```bash
source /opt/ros/humble/setup.bash
```
---

### 2. Buat Workspace

1. Buat direktori untuk workspace:

```bash
mkdir -p ~/ros2_ws/src
```

2. Masuk ke direktori workspace:

```bash
cd ~/ros2_ws
```

---
### 3. Build Workspace

Gunakan alat build `colcon` untuk menginisialisasi dan membangun workspace:

```bash
colcon build
```

---

### 4. Source Workspace

Setelah build selesai, source workspace untuk mengoverlay lingkungan Anda:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 5. Tambahkan Paket ke Workspace

Untuk menambahkan paket ROS 2:

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

## Catatan

- Selalu rebuild workspace menggunakan `colcon build` setelah menambahkan atau memodifikasi paket.
- Gunakan `source install/setup.bash` pada setiap sesi terminal baru sebelum bekerja dengan workspace.

---

## Sumber Daya

Untuk tutorial lebih rinci, kunjungi [dokumentasi resmi ROS 2](https://docs.ros.org/en/rolling/index.html).
