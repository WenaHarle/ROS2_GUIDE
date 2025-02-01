# ROS 2: Service Client dalam Python 

## 1. Pendahuluan

BAB ini akan berfokus pada penggunaan **Service Client** dalam Python. Kita akan mengimplementasikan client untuk mengubah warna garis yang ditinggalkan oleh TurtleSim berdasarkan posisinya di layar.

## 2. Konsep Dasar ROS 2 Services

### 2.1 Mengapa Menggunakan Services?
Di ROS 2, komunikasi antar-node dapat dilakukan melalui **Topics** atau **Services**. Namun, ada perbedaan mendasar antara keduanya:

| Fitur | Topics | Services |
|--------|--------|----------|
| Model komunikasi | Publisher-Subscriber | Client-Server |
| Jenis komunikasi | Streaming data secara terus-menerus | Request-response satu kali |
| Contoh penggunaan | Sensor mengirim data ke beberapa node | Mengubah parameter robot, meminta hasil komputasi |

Topics digunakan untuk **mengirimkan data secara terus-menerus**, sementara Services digunakan untuk **permintaan satu kali dan mendapatkan respons langsung**.

### 2.2 Cara Kerja Service Client
Sebuah **service server** akan menyediakan layanan tertentu, sedangkan **service client** akan mengirimkan permintaan dan menunggu respons dari server. Struktur data dalam service memiliki dua bagian:
1. **Request**: Data yang dikirim oleh client ke server.
2. **Response**: Data yang dikembalikan oleh server ke client.

## 3. Implementasi Service Client dengan Python

### 3.1 Menjalankan TurtleSim dan Mengecek Service yang Tersedia

Sebelum membuat client, kita harus menjalankan **TurtleSim** terlebih dahulu:
```bash
ros2 run turtlesim turtlesim_node
```

Kemudian, kita dapat melihat daftar service yang tersedia dengan perintah:
```bash
ros2 service list
```

Salah satu service yang tersedia adalah:
```
/turtle1/set_pen
```

Kita bisa melihat tipe service ini dengan:
```bash
ros2 service type /turtle1/set_pen
```
Output:
```
turtlesim/srv/SetPen
```

Untuk melihat detail dari request dan response, gunakan:
```bash
ros2 interface show turtlesim/srv/SetPen
```
Output:
```
int64 r
int64 g
int64 b
int64 width
int64 off
---
```
Bagian atas adalah **request**, sedangkan bagian setelah "---" adalah **response** (dalam kasus ini kosong karena tidak ada data yang dikembalikan).

### 3.2 Membuat Service Client di Python

Buka direktori workspace Anda dan buat file baru `turtle_pen_client.py` dalam package yang sesuai:

```python
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from functools import partial

class TurtlePenClient(Node):
    def __init__(self):
        super().__init__('turtle_pen_client')

    def call_setpen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Menunggu service tersedia...')
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_response))

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Service dipanggil dengan sukses!')
        except Exception as e:
            self.get_logger().error(f'Service call gagal: {e}')

if __name__ == '__main__':
    rclpy.init()
    node = TurtlePenClient()
    node.call_setpen_service(255, 0, 0, 3, 0)  # Mengubah warna pen menjadi merah
    rclpy.spin(node)
    rclpy.shutdown()
```

Simpan file ini, lalu jalankan dengan perintah berikut:
```bash
ros2 run my_package turtle_pen_client.py
```
Jika berhasil, warna pen akan berubah sesuai dengan parameter yang diberikan.

## 4. Implementasi dalam Kontrol Otomatis

### 4.1 Mengubah Warna Pen Berdasarkan Posisi
Tambahkan kode berikut ke dalam **callback posisi** untuk mendeteksi perubahan posisi TurtleSim:

```python
def pose_callback(self, msg):
    if msg.x > 5.5:
        self.call_setpen_service(255, 0, 0, 3, 0)  # Merah
    else:
        self.call_setpen_service(0, 255, 0, 3, 0)  # Hijau
```

Dengan menambahkan kondisi ini, TurtleSim akan mengganti warna garis berdasarkan posisinya di layar.

## 5. Perbandingan Service, Topics, dan Actions

Selain **Services** dan **Topics**, ROS 2 juga memiliki **Actions**, yang memungkinkan eksekusi proses yang lebih panjang dan dapat diperbarui selama eksekusi berlangsung.

| Komunikasi | Model | Gunakan Jika |
|------------|--------|--------------|
| Topics | Publisher-Subscriber | Mengirim data secara terus-menerus (misalnya sensor) |
| Services | Client-Server | Permintaan satu kali dengan respons cepat (misalnya mengubah parameter) |
| Actions | Client-Server dengan feedback | Proses yang berjalan lama dan memerlukan update (misalnya pergerakan robot) |

## 6. Kesimpulan

- **Services** digunakan untuk komunikasi **request-response** yang tidak kontinu.
- Service Client memungkinkan kita **mengirimkan permintaan dan menerima jawaban**.
- Services cocok untuk tugas yang memerlukan pemrosesan data secara **satu kali**, berbeda dengan **topics** yang berjalan terus-menerus.
- Dalam implementasi ini, kita menggunakan **Service Client untuk mengubah warna pen TurtleSim berdasarkan posisi**.

Dengan memahami perbedaan antara Topics, Services, dan Actions, kita bisa memilih model komunikasi yang paling sesuai untuk aplikasi ROS 2 yang sedang dikembangkan.

---
Semoga panduan ini membantu Anda memahami ROS 2 Services lebih dalam! 🚀

