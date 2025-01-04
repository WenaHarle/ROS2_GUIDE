# Installasi ROS 2 Humble di Ubuntu

Panduan ini menjelaskan langkah-langkah untuk menginstal ROS 2 Humble Hawksbill di Ubuntu dengan menggunakan paket debian (Ubuntu 22.04 direkomendasikan).

## Prasyarat

1. **Ubuntu 22.04 LTS**
   - Pastikan Anda menggunakan Ubuntu 22.04 atau versi yang kompatibel.

2. **Akses Internet**
   - Dibutuhkan untuk mengunduh paket dan dependensi ROS 2.

3. **Hak Akses Administrator**
   - Anda memerlukan hak `sudo` untuk instalasi.

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

