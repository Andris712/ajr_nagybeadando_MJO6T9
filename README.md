# my_beadando_pkg – ROS2 nagybeadandó

Ez a csomag egy egyszerű ROS2 adatfeldolgozó pipeline-t valósít meg.

A rendszer egy véletlen számot generáló szenzort, egy feldolgozó node-ot és
a köztük lévő topic-kommunikációt tartalmazza, launch fájl segítségével indítva.

---

## 📦 Node-ok

### 1. random_sensor
- **Feladat:** Véletlen `float` értékek publikálása
- **Topic:** `/random_value`
- **Üzenet típus:** `std_msgs/msg/Float32`
- **Paraméter:**
  - `period` (float, alapértelmezett: `0.5`)
  - Meghatározza a publikálás időközét másodpercben

### 2. random_processor
- **Feladat:** A szenzor értékeinek feldolgozása
- **Feliratkozik:** `/random_value`
- **Publikál:** `/processed_value`
- **Üzenet típus:** `std_msgs/msg/Float32`

---

## 🔗 Topic-ok

| Topic neve | Típus | Leírás |
|-----------|------|--------|
| `/random_value` | Float32 | Szenzor által generált érték |
| `/processed_value` | Float32 | Feldolgozott kimeneti érték |

---

## 🚀 Indítás (Launch)

A teljes pipeline egyetlen launch fájllal indítható:

```bash
ros2 launch my_beadando_pkg random_pipeline.launch.py

my_beadando_pkg/
├── launch/
│   └── random_pipeline.launch.py
├── my_beadando_pkg/
│   ├── random_sensor_node.py
│   ├── processor_node.py
│   ├── publisher_node.py
│   ├── subscriber_node.py
│   └── __init__.py
├── resource/
├── test/
├── package.xml
├── setup.py
├── setup.cfg
└── README.md

