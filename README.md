# SO-101 Arm: Modelado, Simulación y Teleoperación

Guía de instalación para reproducir el entorno del proyecto. Los comandos y puertos (`COMx`) están escritos para Windows; las diferencias para Linux (Ubuntu) están en la sección [Consideraciones para Linux (Ubuntu)](#consideraciones-para-linux-ubuntu).

## Requisitos previos

- Windows 10/11
- [Git](https://git-scm.com/downloads)
- [Miniforge](https://github.com/conda-forge/miniforge) (o Miniconda)
- Brazos SO-101 (leader y follower) conectados por USB, solo si se va a usar el hardware físico

## 1. Clonar este repositorio

```bash
git clone https://github.com/<tu-usuario>/<tu-repositorio>.git
cd <tu-repositorio>
```

## 2. Crear el entorno de conda

```bash
conda create -y -n lerobot python=3.12
conda activate lerobot
```

> Este paso de activación se repite cada vez que abras una terminal nueva.

## 3. Instalar ffmpeg

```bash
conda install ffmpeg=7.1.1 -c conda-forge
```

## 4. Clonar e instalar LeRobot

LeRobot **no está incluido** en este repositorio; se descarga desde el repositorio oficial. Para asegurar compatibilidad, se fija la versión usada en el proyecto.

```bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot
git checkout <COMMIT_O_TAG>
```

Instalar en modo editable con soporte para los motores Feetech (STS3215) y el manejo de datasets:

```bash
pip install -e ".[feetech]"
pip install -e ".[dataset]"
cd ..
```

## 5. Instalar MuJoCo

```bash
pip install mujoco
```

## 6. Verificar la instalación

```bash
python -c "import lerobot, mujoco; print('Instalación correcta')"
```

## 7. Configurar los puertos USB

Con los brazos conectados, identifica el puerto de cada uno:

```bash
lerobot-find-port
```

Ejecútalo una vez por brazo (desconectando y conectando el cable cuando lo pida). Luego edita la variable de puerto (`COM4`, `COM5`, etc.) al inicio de los scripts que vayas a usar.

## 8. Calibración

Los archivos de calibración incluidos en `calibration_files/` corresponden a los brazos específicos de este proyecto. Si usas **otros brazos**, debes calibrarlos de nuevo:

```bash
lerobot-calibrate --robot.type=so101_follower --robot.port=COM5 --robot.id=so101_follower
lerobot-calibrate --teleop.type=so101_leader --teleop.port=COM4 --teleop.id=so101_leader
```

Después copia los JSON generados a la carpeta `calibration_files/` del proyecto con los nombres `so101_follower.json` y `so101_leader.json`.

## Consideraciones para Linux (Ubuntu)

Los pasos 1 a 6 son los mismos, con las diferencias que se indican a continuación. Esta sección aplica, por ejemplo, a la máquina donde se graban los datasets.

### Dependencias del sistema

Antes de instalar LeRobot, instala las herramientas de compilación y las librerías de ffmpeg necesarias para compilar PyAV:

```bash
sudo apt-get update
sudo apt-get install -y git cmake build-essential pkg-config \
  libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev \
  libswscale-dev libswresample-dev libavfilter-dev
```

### Instalar Miniforge

```bash
wget "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

Cierra y vuelve a abrir la terminal, y continúa con el paso 2.

### Puertos serie

En Linux los brazos aparecen como `/dev/ttyACM0`, `/dev/ttyACM1`, etc., en lugar de `COM4`/`COM5`. Identifica cada puerto con:

```bash
lerobot-find-port
```

Edita las variables de puerto al inicio de los scripts, por ejemplo:

```python
FOLLOWER_PORT = "/dev/ttyACM0"
LEADER_PORT = "/dev/ttyACM1"
```

> El número asignado puede cambiar al reconectar los cables o reiniciar. Verifica el puerto de cada brazo cada vez que cambies la conexión.

### Permisos de acceso al puerto

Si aparece un error de `Permission denied` al abrir el puerto, agrega tu usuario al grupo `dialout`:

```bash
sudo usermod -aG dialout $USER
```

Cierra sesión y vuelve a iniciarla para que el cambio surta efecto. Como solución temporal (se pierde al reconectar el cable):

```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

### Cámaras

Las cámaras aparecen como `/dev/video0`, `/dev/video1`, etc. Cada cámara USB suele registrar **dos** dispositivos (uno de video y otro de metadatos), por lo que los índices pueden no ser consecutivos. Para listarlas:

```bash
sudo apt-get install -y v4l-utils
v4l2-ctl --list-devices
```

Usa ese índice (o la ruta `/dev/videoX`) en `index_or_path` de `camera.py` y en la configuración de grabación. Si hay error de permisos, agrega tu usuario al grupo `video`:

```bash
sudo usermod -aG video $USER
```

### Calibración y archivos de configuración

Los archivos de `calibration_files/` son independientes del sistema operativo y se pueden usar tal cual, siempre que sean los mismos brazos físicos. Lo único que cambia son los puertos.

### Simulación con MuJoCo

El visor de MuJoCo necesita un entorno gráfico. En una máquina de escritorio con Ubuntu funciona directamente. Si accedes por SSH o sin pantalla, el visor interactivo no abrirá; en ese caso usa la máquina solo para teleoperación y grabación, y corre la simulación en otro equipo.

Si el script `fake_leader.py` falla al importar `tkinter`, instala:

```bash
sudo apt-get install -y python3-tk
```

### Grabación de datasets

Para subir los datasets a Hugging Face Hub, inicia sesión una vez en el entorno:

```bash
hf auth login
```

(en versiones anteriores de la herramienta el comando es `huggingface-cli login`). Verifica también que haya espacio en disco suficiente: los datasets con video ocupan varios GB.

## Estructura esperada de carpetas

```
<tu-repositorio>/
├── calibration_files/
│   ├── so101_follower.json
│   └── so101_leader.json
├── model/
│   ├── scene.xml
│   ├── so101_new_calib.xml
│   └── assets/            # mallas .stl del modelo
├── *.py
└── lerobot/               # clonado en el paso 4 (ignorado por git)
```

## Notas

- Siempre ejecuta los scripts desde la raíz del proyecto, con el entorno `lerobot` activado.
- Si hay errores al compilar dependencias, instala las herramientas de compilación de Visual Studio (C++ Build Tools).
