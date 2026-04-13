# CDFR26 ESP32 rebuild

## Build

Lancer le build via le wrapper du projet :

```bash
./scripts/pio_run.sh esp32-s3-devkitm-1
```

Ce script prépare automatiquement l'environnement local du projet :

- création du venv `.venv-pio`
- installation de PlatformIO 6.x
- correction de compatibilité `empy==3.3.4` pour ESP-IDF / micro-ROS
- initialisation des submodules `components/micro_ros_espidf_component` et `components/SparkFun_Qwiic_OTOS_ESP32_Library`
- rebuild forcé de micro-ROS si les artefacts générés sont incomplets ou pas en transport `custom`

La bibliothèque SparkFun OTOS reste utilisée depuis `components/SparkFun_Qwiic_OTOS_ESP32_Library/`.

## Flash

Pour builder puis flasher :

```bash
./scripts/pio_run.sh esp32-s3-devkitm-1 -t upload
```

Le wrapper détecte automatiquement le port série USB de l'ESP32 si une seule carte compatible est branchée.

Si plusieurs ports sont présents, préciser le port :

```bash
./scripts/pio_run.sh esp32-s3-devkitm-1 -t upload --upload-port /dev/ttyACM0
```

## Notes micro-ROS

Le transport micro-ROS est forcé en `custom` via [app-colcon.meta](/home/adembch/Documents/PlatformIO/Projects/CDFR26_ESP32_rebuild/app-colcon.meta).

Sur l'ESP32-S3 de ce projet, le transport custom passe par l'USB série natif (`/dev/ttyACM*` côté hôte), avec un baudrate hôte fixé à `921600` dans [include/config.h](/home/adembch/Documents/PlatformIO/Projects/CDFR26_ESP32_rebuild/include/config.h).

## Nettoyage complet

Si vous voulez régénérer entièrement l'environnement de build :

```bash
rm -rf .pio .venv-pio \
  components/micro_ros_espidf_component/include \
  components/micro_ros_espidf_component/libmicroros.a \
  components/micro_ros_espidf_component/micro_ros_dev \
  components/micro_ros_espidf_component/micro_ros_src \
  components/micro_ros_espidf_component/esp32_toolchain.cmake

./scripts/pio_run.sh esp32-s3-devkitm-1
```
