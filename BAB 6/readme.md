# Mengintip Dalaman Topic

Mungkin pada bab sebelumnya sudah cukup dijelaskan mengenai topic pada Node di Bab kali ini akan dijelaskan tambahan mengenai topic ini tersendiri.

## Prasyarat
- Anda memahami dasar-dasar node dalam ROS 2.

## Mengapa Topik?
Topik memungkinkan komunikasi antara node dalam ROS 2. Node dapat mempublikasikan data ke suatu topik, dan node lain dapat berlangganan untuk menerima data tersebut.

## Pengenalan Mendalam

### 1. Menjalankan Node Publisher dan Subscriber

Jalankan perintah berikut di terminal untuk menjalankan node publisher dan subscriber:

```bash
# Jalankan talker node (publisher)
ros2 run demo_nodes_cpp talker
```
Node ini mempublikasikan pesan "Hello World" dengan angka yang meningkat ke topik `chatter`.

Buka terminal baru:
```bash
# Jalankan listener node (subscriber)
ros2 run demo_nodes_cpp listener
```
Node ini akan menerima dan menampilkan pesan yang dipublikasikan oleh `talker`.

### 2. Visualisasi dengan `rqt_graph`

Gunakan perintah berikut untuk melihat grafik komunikasi antara node:
```bash
rqt_graph
```
Pilih opsi "Nodes and Topics" untuk melihat node `talker`, `listener`, dan topik `chatter`.

### 3. Eksplorasi Topik dengan CLI

#### Daftar Topik
Lihat daftar topik yang sedang aktif:
```bash
ros2 topic list
```

#### Informasi Topik
Periksa detail tentang topik `chatter`:
```bash
ros2 topic info /chatter
```
Informasi ini mencakup:
- Nama topik
- Tipe pesan
- Jumlah publisher dan subscriber

#### Melihat Pesan Secara Langsung
Anda dapat melihat pesan langsung dari terminal:
```bash
ros2 topic echo /chatter
```

### 4. Komunikasi dengan TurtleSim

#### Jalankan Simulasi TurtleSim

```bash
# Jalankan TurtleSim
ros2 run turtlesim turtlesim_node
```

Buka terminal baru:
```bash
# Jalankan kontrol TurtleSim menggunakan keyboard
ros2 run turtlesim turtle_teleop_key
```

Gunakan tombol panah di keyboard untuk menggerakkan kura-kura.

#### Visualisasi dengan `rqt_graph`

```bash
rqt_graph
```
Anda akan melihat dua node (`turtlesim_node` dan `teleop_turtle`) yang berkomunikasi melalui topik `/turtle1/cmd_vel`. Node `teleop_turtle` mempublikasikan ke topik tersebut, sementara `turtlesim_node` berlangganan untuk menerima pesan.

#### Eksplorasi Topik TurtleSim

Informasi tentang topik `/turtle1/cmd_vel`:
```bash
ros2 topic info /turtle1/cmd_vel
```

Lihat isi pesan dari topik tersebut:
```bash
ros2 topic echo /turtle1/cmd_vel
```
Pesan yang dikirim memiliki tipe `geometry_msgs/msg/Twist` dengan data:
- Linear (x, y, z)
- Angular (x, y, z)

## Penutup

Topik memungkinkan komunikasi anonim antar node, di mana node dapat mempublikasikan dan/atau berlangganan tanpa mengetahui detail node lainnya. Dengan memahami konsep ini, Anda dapat membangun aplikasi ROS 2 yang kompleks dan terhubung.

