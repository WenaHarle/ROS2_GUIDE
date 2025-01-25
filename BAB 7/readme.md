# ROS 2 Publisher Node - Draw Circle

Tutorial ini menjelaskan cara membuat dan menjalankan node ROS 2 dalam Python untuk mengontrol TurtleSim agar menggambar lingkaran. Node ini mencakup pembuatan publisher yang mengirimkan pesan ke topik untuk mengendalikan pergerakan TurtleSim.

---

## Prasyarat

Pastikan Anda telah menginstal:

1. **ROS 2** (Disarankan menggunakan ROS 2 Humble atau yang lebih baru).
2. **TurtleSim**
3. **Workspace ROS 2 yang terkonfigurasi dengan benar** (misalnya: `colcon` untuk build dan `setup.py`).

---

## Langkah-langkah Implementasi

### 1. Buat Node Python

Pindah ke direktori package:

```bash
cd ~/ros2_ws/src/my_robot_controller
```

Buat file Python:

```bash
touch draw_cycle.py
chmod +x draw_cycle.py
```

Isi file `draw_cycle.py` dengan kode berikut:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCycleNode(Node):
    def __init__(self):
        super().__init__('draw_cycle')
        self.get_logger().info('DrawCycleNode has been started.')

        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Create timer
        self.create_timer(0.5, self.send_velocity_command)

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub.publish(msg)

        self.get_logger().info(f'Published message: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = DrawCycleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Penjelasan Baris demi Baris Kode Program

- `#!/usr/bin/env python3`:
  Shebang untuk menentukan interpreter Python yang digunakan.

- `import rclpy`:
  Import modul utama ROS 2 untuk Python.

- `from rclpy.node import Node`:
  Mengimpor kelas dasar untuk membuat node di ROS 2.

- `from geometry_msgs.msg import Twist`:
  Mengimpor jenis pesan `Twist` untuk mengendalikan kecepatan linear dan angular TurtleSim.

- `class DrawCycleNode(Node):`:
  Mendefinisikan kelas `DrawCycleNode` yang merupakan turunan dari kelas ROS 2 `Node`.

- `def __init__(self):`:
  Konstruktor untuk inisialisasi node.

- `super().__init__('draw_cycle')`:
  Memanggil konstruktor kelas induk untuk membuat node bernama "draw_cycle".

- `self.get_logger().info('DrawCycleNode has been started.')`:
  Menampilkan log informasi bahwa node telah dimulai.

- `self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)`:
  Membuat publisher untuk mengirim pesan ke topik `/turtle1/cmd_vel` dengan queue size 10.

- `self.create_timer(0.5, self.send_velocity_command)`:
  Membuat timer yang memanggil fungsi `send_velocity_command` setiap 0.5 detik.

- `def send_velocity_command(self):`:
  Fungsi untuk mengirim perintah kecepatan.

- `msg = Twist()`:
  Membuat instance pesan tipe `Twist`.

- `msg.linear.x = 2.0`:
  Menetapkan kecepatan linear TurtleSim sebesar 2.0.

- `msg.angular.z = 1.0`:
  Menetapkan kecepatan angular TurtleSim sebesar 1.0.

- `self.cmd_vel_pub.publish(msg)`:
  Menerbitkan pesan ke topik yang ditentukan.

- `self.get_logger().info(f'Published message: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')`:
  Menampilkan log dengan informasi pesan yang diterbitkan.

- `def main(args=None):`:
  Fungsi utama program.

- `rclpy.init(args=args)`:
  Menginisialisasi ROS 2.

- `node = DrawCycleNode()`:
  Membuat instance node `DrawCycleNode`.

- `rclpy.spin(node)`:
  Menjalankan loop ROS untuk mendengarkan callback atau timer.

- `node.destroy_node()`:
  Menghapus instance node.

- `rclpy.shutdown()`:
  Menonaktifkan sistem ROS 2.

- `if __name__ == '__main__':`:
  Menentukan titik awal eksekusi program.

### 2. Tambahkan Dependensi pada `package.xml`

Edit `package.xml` untuk menambahkan dependensi:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>turtlesim</depend>
```

### 3. Tambahkan Node di `setup.py`

Edit `setup.py` untuk menambahkan script:

```python
entry_points={
    'console_scripts': [
        'draw_cycle = my_robot_controller.draw_cycle:main',
    ],
},
```

### 4. Build Workspace

Kembali ke root workspace, build project, dan gunakan opsi `symlink`:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 5. Jalankan Node

#### a. Jalankan TurtleSim Node

```bash
ros2 run turtlesim turtlesim_node
```

#### b. Jalankan `draw_cycle` Node

```bash
source install/setup.bash
ros2 run my_robot_controller draw_cycle
```

Turtle akan mulai menggambar lingkaran di jendela TurtleSim.

---

## Debugging dan Verifikasi

### List Topik

```bash
ros2 topic list
```

### Info Topik

```bash
ros2 topic info /turtle1/cmd_vel
```

### Data Topik

```bash
ros2 topic echo /turtle1/cmd_vel
```

---

## Hasil

Setelah menjalankan `draw_cycle`, TurtleSim akan menggambar lingkaran secara otomatis.

---

Selamat mencoba! Jika Anda mengalami kendala, jangan ragu untuk menghubungi komunitas ROS 2 atau memeriksa dokumentasi resmi.
