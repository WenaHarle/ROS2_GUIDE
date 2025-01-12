# Salam Kenal Node

## Pengenalan Node ROS 2

### Apa itu Node ROS 2?
Node ROS 2 adalah program yang terhubung ke lingkungan ROS 2. Node digunakan untuk melakukan berbagai tugas, seperti:
- Mempublikasikan pesan.
- Berlangganan pesan.
- Mengelola perangkat keras atau antarmuka eksternal.
- Membuat antarmuka grafis.

Node-node ini saling berkomunikasi menggunakan **topik**, **layanan**, dan **aksi**, yang bersama-sama membentuk grafik ROS (**ROS graph**). Grafik ini adalah representasi dari semua node dan komunikasi di lingkungan ROS.


---

## Alat yang Dibahas

1. **Alat Command-line:**
   - `ros2 run`: Menjalankan node dari paket yang telah terinstal.
   - `rqt_graph`: Memvisualisasikan grafik ROS.

2. **Node Contoh yang Sudah Terinstal:**
   - Paket: `demo_nodes_cpp`
   - Node: `talker`, `listener`

3. **Paket TurtleSim:**
   - `turtlesim_node`: Simulasi 2D dengan tampilan grafis.
   - `turtle_teleop_key`: Node teleoperation menggunakan keyboard.

---

## Contoh yang Dibahas di Episode 2

### Menjalankan Node Contoh
1. **Node Talker**:
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
    - Mempublikasikan pesan "Hello World" dengan penghitung.

2. **Node Listener**:
    ```bash
    ros2 run demo_nodes_cpp listener
    ```
    - Berlangganan pesan yang dipublikasikan oleh node `talker`.

3. **Memvisualisasikan Komunikasi:**
    - Jalankan `rqt_graph` untuk melihat node dan topik dalam grafik ROS.
    ```bash
    rqt_graph
    ```

### Contoh TurtleSim
1. **Jalankan Node TurtleSim:**
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    - Meluncurkan simulasi grafis dengan seekor kura-kura.

2. **Kendali Kura-kura:**
    ```bash
    ros2 run turtlesim turtle_teleop_key
    ```
    - Gunakan tombol panah pada keyboard untuk menggerakkan kura-kura dalam simulasi.

3. **Amati Komunikasi:**
    - Visualisasikan interaksi antara teleop dan turtlesim menggunakan `rqt_graph`.

---

## Langkah Selanjutnya
Di tutorial berikutnya, kita akan membahas:
1. Membuat node ROS 2 kustom Anda sendiri.
2. Menulis program sederhana untuk publisher dan subscriber dalam Python dan C++.
3. Memahami konsep topik di ROS 2 secara mendalam.

---

## Sumber Tambahan
Jika Anda menikmati episode ini, Anda mungkin tertarik dengan kursus lengkap **ROS 2 untuk Pemula**. Kursus ini mencakup lebih dari 10 jam konten yang akan membantu Anda membangun aplikasi ROS 2 secara lengkap. Lihat tautannya di deskripsi video!

### Tetap Terhubung
- **Playlist:** [Seri Playlist](#)  
- **Episode Berikutnya:** [Buat Node ROS 2 Anda Sendiri](#)  

Terima kasih telah mengikuti tutorial ini. Sampai jumpa di tutorial berikutnya!

