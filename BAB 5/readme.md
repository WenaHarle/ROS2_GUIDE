# Salam Kenal Node

## Pengenalan Node ROS 2

Node adalah unit utama dalam sistem **Robot Operating System 2 (ROS 2)**. Setiap node berjalan sebagai program independen yang dapat bekerja secara terpisah maupun bersama dengan node lainnya untuk membentuk suatu sistem yang kompleks. Berikut adalah tugas-tugas utama dari sebuah node dalam ROS 2:

### Tugas Node dalam ROS 2

1. **Mempublikasikan Pesan**  
   Node dapat mengirimkan data (informasi) ke node lain melalui **topik** tertentu. Proses ini disebut sebagai _publishing_, dan data yang dikirimkan dapat berupa teks, angka, sensor, gambar, dan lain-lain.

2. **Berlangganan Pesan**  
   Node dapat menerima data dari node lain melalui **topik** yang telah mereka daftarkan. Proses ini disebut sebagai _subscribing_. Berlangganan memungkinkan node untuk berinteraksi atau merespon data dari node lain.

3. **Mengelola Perangkat Keras atau Antarmuka Eksternal**  
   Node sering digunakan untuk berkomunikasi langsung dengan perangkat keras, seperti sensor, aktuator, atau pengendali. Node bertugas mengambil data dari perangkat keras atau mengirimkan perintah untuk menggerakkan perangkat.

4. **Menyediakan atau Menggunakan Layanan (Services)**  
   Node dapat menyediakan layanan (_service_), yaitu fungsi yang dapat dipanggil oleh node lain untuk melakukan tugas tertentu. Contohnya adalah perhitungan sederhana atau kontrol perangkat.

5. **Berinteraksi melalui Aksi (Actions)**  
   Node dapat mengatur atau menjalankan aksi (_action_), yaitu tugas yang lebih kompleks dan memakan waktu. Aksi biasanya digunakan dalam skenario seperti navigasi robot atau manipulasi lengan robotik.

6. **Membuat Antarmuka Grafis**  
   Node dapat bertugas membangun **Graphical User Interface (GUI)** yang memudahkan pengguna untuk berinteraksi dengan sistem ROS 2, misalnya untuk visualisasi data atau kontrol perangkat keras.

7. **Mengatur Logika Sistem atau Pemrosesan Data**  
   Node dapat diprogram untuk menangani logika pemrosesan, seperti filter data sensor, algoritma kontrol, hingga penerapan _machine learning_.
   
---
---

## Komunikasi Publisher-Subscriber dengan Topic pada ROS

Dalam ROS (Robot Operating System), komunikasi antar-node dilakukan melalui mekanisme yang disebut *topics*. *Topics* memungkinkan pengiriman dan penerimaan data antar-*nodes* tanpa mereka harus terhubung secara langsung. Berikut adalah penjelasan detail mengenai konsep ini dan elemen-elemen utamanya:

---

### **Elemen Dasar**

#### **1. Apa itu Publisher?**
*Publisher* adalah bagian dari *node* (program) yang bertanggung jawab untuk **mengirimkan data atau pesan** ke sistem ROS melalui sebuah *topic*. 
- Ibarat orang yang berbicara di sebuah saluran radio.
- **Contoh**: Sebuah *node* sensor pengukur suhu mengirimkan data suhu melalui *topic* bernama `/temperature`.

#### **2. Apa itu Subscriber?**
*Subscriber* adalah bagian dari *node* yang bertugas untuk **menerima data atau pesan** dari sistem ROS melalui sebuah *topic* yang spesifik.
- Ibarat orang yang mendengarkan saluran radio tertentu untuk mendapatkan informasi.
- **Contoh**: Sebuah *node* pengontrol membaca data suhu dari *topic* `/temperature` untuk mengambil keputusan, seperti menyalakan kipas.

#### **3. Apa itu Topic?**
*Topic* adalah saluran komunikasi yang digunakan oleh ROS untuk **menghubungkan publisher dengan subscriber**. 
- Ibarat saluran radio tempat berbagi informasi.
- *Publisher* mengirimkan pesan ke sebuah *topic*, sedangkan *subscriber* akan menerima pesan dari *topic* tersebut.
- Nama *topic* biasanya dalam format string unik seperti `/velocity`, `/camera/image`, atau `/lidar/scan`.

---

### **Bagaimana Publisher, Subscriber, dan Topic Bekerja Bersama?**

Komunikasi melalui *topics* tidak hanya bersifat satu ke satu (*point-to-point*), tetapi juga mendukung pola komunikasi berikut:

1. **Satu ke Banyak** (*One-to-Many*):
   - Satu *node* bertindak sebagai *publisher* dan mengirimkan pesan yang dapat diterima oleh beberapa *nodes* lain sebagai *subscribers*.
   
2. **Banyak ke Satu** (*Many-to-One*):
   - Beberapa *nodes* menjadi *publishers* dan mengirimkan data ke satu *node* yang menjadi *subscriber*.

3. **Banyak ke Banyak** (*Many-to-Many*):
   - Kombinasi di mana banyak *nodes* berperan sebagai *publishers* dan *subscribers* sekaligus.

#### **Contoh Kehidupan Sehari-hari:**
Bayangkan sebuah stasiun radio (*topic*):
- **Publisher:** Penyiar radio berbicara di saluran (misalnya, frekuensi 101.1 FM).
- **Subscriber:** Pendengar radio yang menyetel saluran 101.1 FM untuk mendengarkan.
- Semua orang yang mendengarkan frekuensi tersebut menerima pesan yang sama dari penyiar.

Dalam ROS, analoginya adalah pesan yang diterbitkan ke sebuah *topic* dapat "didengar" oleh banyak *subscriber*, atau beberapa *publisher* dapat mengirim pesan ke *topic* yang sama.

---
---
## **Contoh Ilustrasi**
![GIF Description](images/TP.gif)

GIF menunjukkan bagaimana sebuah sistem komunikasi berbasis *topic* bekerja:

1. Sebuah *node* yang bertindak sebagai **publisher** mengirimkan pesan melalui *topic* tertentu.
2. Beberapa *nodes* lain yang berperan sebagai **subscribers** telah terhubung ke *topic* yang sama.
3. **Proses Pengiriman:**
   - Pesan yang diterbitkan oleh *publisher* dikirimkan melalui *topic*.
   - Semua *subscriber* yang "berlangganan" ke *topic* tersebut menerima pesan secara bersamaan.

### **Pola Komunikasi:**
- **Satu Publisher, Banyak Subscriber:** 
  - Sebuah *publisher* menerbitkan pesan, yang diterima oleh dua *subscriber*.
- Mekanisme ini memvisualisasikan fleksibilitas komunikasi *topics* dalam mendukung berbagai skenario seperti distribusi data sensor atau penyebaran perintah kontrol. seingga memungkikan dalam melakukan multitasking tnpa menghentikan perintah lain.

> Dengan memahami *topics*, *publishers*, dan *subscribers*, kita dapat merancang sistem robotik yang modular dan efisien, di mana komponen-komponennya dapat saling berkomunikasi secara asinkron tanpa terhubung secara langsung satu sama lain.
---
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
---
## Contoh 

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

