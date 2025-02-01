# Panduan Lengkap ROS 2 Service Client dengan Python

## Apa itu ROS 2 Service?

**Service** dalam ROS 2 adalah mekanisme komunikasi yang memungkinkan interaksi **client-server**. Tidak seperti **Topics**, yang memungkinkan komunikasi **one-way (satu arah)**, Service memungkinkan komunikasi **dua arah** dengan format **request-response**.

**Ciri-ciri ROS 2 Services:**

- **Client** mengirim permintaan (**request**) ke **Server**.
- **Server** memproses permintaan dan mengembalikan jawaban (**response**).
- **Client harus menunggu hingga Server mengirimkan response**.

**Perbandingan Service dengan Topics dan Actions:**

| Fitur             | Topics                         | Services                                 | Actions                                |
| ----------------- | ------------------------------ | ---------------------------------------- | -------------------------------------- |
| Pola Komunikasi   | Publish-Subscribe              | Request-Response                         | Goal-Feedback-Result                   |
| Jumlah Pengirim   | Banyak Publisher               | 1 Client                                 | 1 Client                               |
| Jumlah Penerima   | Banyak Subscriber              | 1 Server                                 | 1 Server                               |
| Waktu Eksekusi    | Real-time, terus-menerus       | Hanya saat diperlukan                    | Bisa berjalan lama dengan feedback     |
| Contoh Penggunaan | Data sensor, kontrol kecepatan | Operasi matematis, perubahan konfigurasi | Pergerakan robot dengan durasi panjang |

---

## Implementasi Service Client dengan Python

### Tujuan

Kita akan mengubah warna garis yang digambar oleh **TurtleSim**, tergantung pada posisinya:

- **Jika kura-kura berada di sisi kanan layar (x > 5.5), warna merah.**
- **Jika kura-kura berada di sisi kiri layar (x <= 5.5), warna hijau.**

### Service yang Digunakan

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

---

## Implementasi Service Client dalam Python

### Persiapan Paket

Pindah ke direktori package:

```bash
cd ~/ros2_ws/src/my_robot_controller/my_robot_controller
```

Buat file `turtle_pen.py`:

```bash
touch turtle_pen.py
chmod +x turtle_pen.py
```

Buka file Python dengan VS-CODE:

```bash
cd ~/ros2_ws/src/my_robot_controller
code .
```
---

### Kode Lengkap untuk Service Client :

Isi file `turtle_pen.py` :

```python
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from functools import partial

class TurtlePen(Node):
    def __init__(self):
        super().__init__('turtle_pen')
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.previous_x = 0.0

    def pose_callback(self, msg):
        # Mengecek apakah kura-kura berpindah dari sisi kiri ke kanan atau sebaliknya
        if (msg.x > 5.5 and self.previous_x <= 5.5) or (msg.x <= 5.5 and self.previous_x > 5.5):
            color = (255, 0, 0) if msg.x > 5.5 else (0, 255, 0)
            self.call_set_pen_service(*color, width=3, off=0)
        self.previous_x = msg.x

    def call_set_pen_service(self, r, g, b, width, off):
        # Membuat klien untuk mengakses service /turtle1/set_pen
        client = self.create_client(SetPen, '/turtle1/set_pen')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for service /turtle1/set_pen...')
        
        # Membuat request untuk mengubah warna dan ketebalan pen
        request = SetPen.Request()
        request.r, request.g, request.b, request.width, request.off = r, g, b, width, off
        
        # Mengirim permintaan ke service secara asinkron
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
            self.get_logger().info('Pen color changed successfully!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

# Fungsi utama untuk menjalankan node

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Penjelasan Kode

#### Header
```python
#!/usr/bin/env python3
```
Shebang yang menentukan interpreter Python 3 secara eksplisit.

---

#### Import Library
```python
import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from functools import partial
```
- **`rclpy`**: Library utama untuk mengembangkan aplikasi ROS 2 menggunakan Python.
- **`Node`**: Kelas dasar dalam ROS 2 untuk membuat node.
- **`SetPen`**: Tipe service yang digunakan untuk mengubah warna dan ketebalan garis yang digambar oleh kura-kura.
- **`Pose`**: Tipe pesan yang digunakan untuk mendapatkan informasi posisi dan orientasi kura-kura di Turtlesim.
- **`partial`**: Fungsi dari `functools` untuk mengikat beberapa argumen ke dalam callback.

---

#### Kelas `TurtlePen`
```python
class TurtlePen(Node):
    def __init__(self):
        super().__init__('turtle_pen')
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.previous_x = 0.0
```
1. **`super().__init__('turtle_pen')`**  
   - Menginisialisasi node dengan nama `turtle_pen`.

2. **`self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)`**  
   - Membuat subscriber untuk menerima data posisi kura-kura dari topik `/turtle1/pose`.  
   - Ketika ada pesan baru, fungsi `pose_callback` akan dipanggil.

3. **`self.previous_x = 0.0`**  
   - Menyimpan posisi x sebelumnya dari kura-kura untuk mendeteksi perubahan posisi.

---

#### Callback `pose_callback`
```python
def pose_callback(self, msg):
    if (msg.x > 5.5 and self.previous_x <= 5.5) or (msg.x <= 5.5 and self.previous_x > 5.5):
        color = (255, 0, 0) if msg.x > 5.5 else (0, 255, 0)
        self.call_set_pen_service(*color, width=3, off=0)
    self.previous_x = msg.x
```
1. **`if (msg.x > 5.5 and self.previous_x <= 5.5) or (msg.x <= 5.5 and self.previous_x > 5.5):`**  
   - Mengecek apakah kura-kura berpindah dari sisi kiri ke kanan (x > 5.5) atau sebaliknya.  
   - Jika iya, maka warna pena akan berubah.

2. **`color = (255, 0, 0) if msg.x > 5.5 else (0, 255, 0)`**  
   - Jika kura-kura berada di sebelah kanan (x > 5.5), warna pena diubah menjadi merah `(255, 0, 0)`.  
   - Jika di sebelah kiri, warna pena diubah menjadi hijau `(0, 255, 0)`.

3. **`self.call_set_pen_service(*color, width=3, off=0)`**  
   - Memanggil fungsi `call_set_pen_service` untuk mengubah warna dan ketebalan pena.

4. **`self.previous_x = msg.x`**  
   - Memperbarui posisi x sebelumnya agar dapat mendeteksi perpindahan pada iterasi berikutnya.

---

#### Fungsi `call_set_pen_service`
```python
def call_set_pen_service(self, r, g, b, width, off):
    client = self.create_client(SetPen, '/turtle1/set_pen')
    while not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().warn('Waiting for service /turtle1/set_pen...')
    
    request = SetPen.Request()
    request.r, request.g, request.b, request.width, request.off = r, g, b, width, off
    
    future = client.call_async(request)
    future.add_done_callback(partial(self.callback_set_pen))
```
1. **`client = self.create_client(SetPen, '/turtle1/set_pen')`**  
   - Membuat klien untuk mengakses service `/turtle1/set_pen`.

2. **`while not client.wait_for_service(timeout_sec=1.0):`**  
   - Menunggu hingga service `/turtle1/set_pen` tersedia.

3. **`request = SetPen.Request()`**  
   - Membuat request untuk mengubah warna dan ketebalan pena.

4. **`request.r, request.g, request.b, request.width, request.off = r, g, b, width, off`**  
   - Menetapkan parameter request berdasarkan warna dan ketebalan pena.

5. **`future = client.call_async(request)`**  
   - Mengirim permintaan ke service secara asinkron.

6. **`future.add_done_callback(partial(self.callback_set_pen))`**  
   - Menambahkan callback untuk menangani respons dari service.

---

#### Callback `callback_set_pen`
```python
def callback_set_pen(self, future):
    try:
        response = future.result()
        self.get_logger().info('Pen color changed successfully!')
    except Exception as e:
        self.get_logger().error(f'Service call failed: {e}')
```
1. **`response = future.result()`**  
   - Mengambil hasil dari permintaan service.

2. **`self.get_logger().info('Pen color changed successfully!')`**  
   - Menampilkan pesan bahwa warna pena berhasil diubah.

3. **`except Exception as e:`**  
   - Jika terjadi kesalahan, menampilkan error pada log.

---

#### Fungsi Utama
```python
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```
1. **`rclpy.init(args=args)`**  
   - Menginisialisasi sistem ROS 2.

2. **`node = TurtlePen()`**  
   - Membuat instance dari kelas `TurtlePen`.

3. **`rclpy.spin(node)`**  
   - Menjalankan node agar tetap aktif.

4. **`node.destroy_node()`**  
   - Menghapus node setelah selesai digunakan.

5. **`rclpy.shutdown()`**  
   - Menonaktifkan sistem ROS 2.

---


#### Kondisi Eksekusi
```python
if __name__ == '__main__':
    main()
```
- Memastikan bahwa kode hanya dieksekusi jika file dijalankan langsung, bukan diimpor sebagai modul.

### Tambahkan Dependensi pada `package.xml`

Edit `package.xml` untuk menambahkan dependensi:

```xml
<depend>rclpy</depend>
<depend>turtlesim</depend>
```

---

### Tambahkan Node di `setup.py`

Edit `setup.py` untuk menambahkan script:

```python
entry_points={
    'console_scripts': [
        'turtle_pen = my_robot_controller.pose_subscriber:main',
    ],
},
```

---

### Build Workspace

Kembali ke root workspace, build project, dan gunakan opsi `symlink`:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

---

### Menjalankan Node Service Client

Setelah kita selesai membuat dan mengatur semua hal yang diperlukan, saatnya menjalankan node tersebut!

1. **Pastikan TurtleSim sedang berjalan:**

    Jika belum menjalankan, buka terminal baru dan jalankan:

    ```bash
    ros2 run turtlesim turtlesim_node
    ```

2. **Jalankan node Python yang kita buat:**

    Kembali ke terminal sebelumnya, jalankan node dengan perintah berikut:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run my_robot_controller turtle_pen
    ```

3. **Uji Node:**

    - Cobalah menggerakkan kura-kura dengan mengirim command melalui keyboard atau dengan teleop:
    
        ```bash
        ros2 run turtlesim turtle_teleop_key
        ```
    
    - Perhatikan bagaimana warna pen berubah saat kura-kura bergerak dari kiri ke kanan atau sebaliknya!

4. **Log Output:**

    Jika node berhasil berjalan, kamu akan melihat log seperti berikut di terminal:

    ```
    [INFO] [<timestamp>] [turtle_pen]: Pen color changed successfully!
    ```

    Jika ada masalah, misalnya service tidak ditemukan, log error akan muncul:

    ```
    [ERROR] [<timestamp>] [turtle_pen]: Service call failed: <error_message>

### Hasil
   
   <p align="center">
     <img src="images/subcriber_turtle.png" alt="workspace" />
   </p>

Setelah menjalankan `pose_subscriber`, Anda akan menerima data posisi dan orientasi TurtleSim secara real-time.

---

