# my_beadando_pkg – kis beadandó

Egyszerű ROS2 (Python) csomag egy publisherrel és egy subscriberrel.  
A publisher fél másodpercenként üzen a **/kisbeadando_chatter** topikra,  
a subscriber kiírja a kapott üzeneteket.

## Követelmények
- ROS 2 (Humble vagy kompatibilis)
- Python 3
- `colcon`

## Telepítés és build (friss klónból)

```bash
# a repó gyökere: sem_mjo_karakum
git clone https://github.com/Andris712/sem_mjo_karakum.git
cd sem_mjo_karakum

# buildeld csak ezt a csomagot
colcon build --packages-select my_beadando_pkg

# forrásold a környezetet
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash










