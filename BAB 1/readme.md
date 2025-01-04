# ROS (Robot Operating System)

## 1.1 Apa itu ROS? 

ROS (Robot Operating System) adalah platform sumber terbuka (Open Source) yang merupakan sistem operasi untuk robot. ROS menyediakan berbagai layanan yang biasa ditemukan di sistem operasi, seperti:

- **Abstraksi perangkat keras**: Mempermudah robot memahami perangkat kerasnya.
- **Kontrol perangkat tingkat rendah**: Mengendalikan motor, sensor, dll.
- **Implementasi fitur-fitur yang sering digunakan**: Misalnya navigasi dan manipulasi.
- **Pengiriman pesan antar proses**: Komunikasi antara program.
- **Manajemen paket**: Mengelola komponen perangkat lunak.

Selain itu, ROS juga dilengkapi alat dan pustaka untuk menulis, menjalankan, dan mengatur kode di banyak komputer, membuatnya sangat fleksibel untuk digunakan di berbagai jenis robot.

ROS menggunakan jaringan proses yang saling berkomunikasi dalam bentuk grafik runtime, mirip dengan "peta" yang menghubungkan berbagai bagian sistem robot. Grafik ini:

- Bisa digunakan di banyak komputer untuk mendistribusikan tugas.
- Menggunakan beberapa cara komunikasi, seperti:
  - **Node**: Untuk mengirim data terus-menerus, misalnya sensor kamera.
  - **Topic**: Untuk meminta tugas tertentu, seperti mengambil data posisi.
  - **Server Parameter**: Untuk menyimpan pengaturan yang sering digunakan.

Jika Anda penasaran, grafik ini dijelaskan lebih dalam di *Tinjauan Konseptual ROS*.

ROS bukan sistem real-time. Namun, ROS dapat bekerja dengan kode waktu nyata. Contohnya, robot PR2 menggunakan sistem bernama `pr2_etherCAT` untuk menghubungkan pesan ROS dengan proses waktu nyata. Selain itu, ROS juga dapat diintegrasikan dengan Orocos Real-time Toolkit, membuatnya bisa menjalankan sistem yang memerlukan ketepatan tinggi.

---

## 1.2 Prinsip Desain Utama ROS

### Kerangka Distribusi:

- ROS adalah kerangka kerja yang terdiri dari proses-proses terdistribusi, yang disebut **Node**. Node-node ini memungkinkan program dieksekusi secara independen tetapi tetap dapat berkomunikasi satu sama lain saat dijalankan.
- Proses-proses tersebut dikelompokkan dalam **Package** dan **Stack** yang mudah dibagikan dan didistribusikan.
- ROS juga mendukung sistem repositori kode terfederasi, memungkinkan kolaborasi lebih luas secara terdistribusi.

### Sederhana (Thin):

- ROS dirancang sesederhana mungkin, tanpa mengubah fungsi utama program Anda (misalnya, `main()`). Dengan cara ini, kode yang ditulis untuk ROS tetap dapat digunakan dengan platform robotik lainnya.
- Sebagai bukti, ROS telah terintegrasi dengan OpenRAVE, Orocos, dan Player.

### Perpustakaan Bebas-ROS:

- Model pengembangan yang disarankan adalah menulis pustaka (library) yang tidak tergantung langsung pada ROS, tetapi tetap memiliki antarmuka fungsi yang bersih dan fleksibel.

### Independen Bahasa Pemrograman:

- ROS dapat digunakan dengan berbagai bahasa pemrograman modern. Saat ini, ROS sudah mendukung Python, C++, dan Lisp, serta memiliki pustaka eksperimental untuk Java dan Lua.

### Pengujian yang Mudah:

- ROS memiliki kerangka uji bawaan bernama `rostest`, yang mempermudah pembuatan dan penghapusan uji coba (*unit test* dan *integration test*).

### Dapat Diskalakan:

- ROS dirancang untuk digunakan pada sistem runtime yang besar serta proyek pengembangan berskala besar.

---

## 1.3 Apa yang Membuat ROS Unik?

ROS memungkinkan Anda tetap memanfaatkan banyak pustaka dan alat yang telah disediakan. Selain itu, karena sifatnya yang ringan, independen bahasa, dan mudah diintegrasikan, ROS dapat bekerja berdampingan dengan platform lain.

Sebagai contoh, Brian Gerkey (pengembang Player dan ROS) pernah menjelaskan dalam sebuah email tentang perbedaan antara ROS dan Player, termasuk integrasi dengan OpenCV. Hal ini menunjukkan fleksibilitas ROS dalam berkolaborasi dengan teknologi robotik lainnya.
