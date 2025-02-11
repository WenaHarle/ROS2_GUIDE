# Unified Robot Description Format (URDF)

## Apa itu URDF?
Unified Robot Description Format (URDF) adalah format berbasis XML yang digunakan dalam ROS (Robot Operating System) untuk mendeskripsikan model robot. URDF memungkinkan pengguna untuk mendefinisikan geometri, hubungan antar link, material, sensor, dan aktuator dari sebuah robot.

## Fungsi URDF
- **Mendefinisikan Struktur Robot**: Menggambarkan bagian-bagian robot dalam bentuk link dan joint.
- **Simulasi dan Visualisasi**: Digunakan dalam simulasi menggunakan RViz dan Gazebo.
- **Integrasi dengan MoveIt!**: Untuk keperluan perencanaan lintasan dan kontrol gerak robot.
- **Custom Plugin**: Dapat digunakan untuk menambahkan sensor atau aktuator kustom dengan plugin tambahan.

## Contoh URDF Sederhana
Berikut adalah contoh URDF sederhana untuk sebuah robot dengan satu link:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
    <!-- Definisi Link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.5 0.5 0.5"/>
            </geometry>
            <material name="blue">
                <color rgba="0.0 0.0 1.0 1.0"/>
            </material>
        </visual>
    </link>
</robot>
```

## Struktur Dasar URDF
1. **`<robot>`**: Elemen utama yang mendefinisikan robot.
2. **`<link>`**: Mewakili bagian dari robot (badan, lengan, roda, dll.).
3. **`<joint>`**: Menghubungkan dua link dan mendefinisikan gerakannya.
4. **`<visual>`**: Menentukan bagaimana robot ditampilkan dalam simulasi.
5. **`<collision>`**: Menentukan bagaimana robot berinteraksi dengan lingkungan dalam simulasi fisik.
6. **`<inertial>`**: Menentukan sifat inersia dari robot untuk simulasi fisika.

## Generate URDF dari 3D model dengan fusion 360

