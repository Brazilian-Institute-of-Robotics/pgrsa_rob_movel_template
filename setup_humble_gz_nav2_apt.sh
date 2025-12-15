#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="humble"
WS_DIR="${1:-pgrsa_ws}"     # passe outro caminho se quiser
USE_OSRF_REPO="${2:-false}"      # true para ativar repo OSRF (ignition-fortress extra)

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WS_PATH="${SCRIPT_DIR}/${WS_DIR}"



echo "==> ROS_DISTRO=$ROS_DISTRO"
echo "==> WS_DIR=$WS_DIR"
echo "==> USE_OSRF_REPO=$USE_OSRF_REPO"

if [[ $EUID -ne 0 ]]; then SUDO="sudo"; else SUDO=""; fi

# Detecta codinome/arch
. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-jammy}}"
ARCH="$(dpkg --print-architecture)"

# --------------------------------------------------------------------
# 1) Pré-requisitos + habilitar Universe (como na doc)
# --------------------------------------------------------------------
$SUDO apt-get update
DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y \
  software-properties-common
$SUDO add-apt-repository -y universe
DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y \
  curl gnupg lsb-release ca-certificates \
  git python3-pip python3-venv \
  build-essential cmake

# --------------------------------------------------------------------
# 2) Configurar repo do ROS 2 via ros2-apt-source (conforme documentação)
#    - baixar .deb da ÚLTIMA release e instalar
# --------------------------------------------------------------------
$SUDO apt-get update && $SUDO apt-get install -y curl
export ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
$SUDO dpkg -i /tmp/ros2-apt-source.deb

# Atualiza caches e recomenda atualizar antes de instalar ROS
$SUDO apt-get update
DEBIAN_FRONTEND=noninteractive $SUDO apt-get -y upgrade

# --------------------------------------------------------------------
# 3) Instalar ROS 2 Humble (desktop) + ferramentas
# --------------------------------------------------------------------
DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y \
  "ros-${ROS_DISTRO}-desktop" \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-dev-tools

# --------------------------------------------------------------------
# 4) (Opcional) Repositório OSRF (para extras do Fortress)
# --------------------------------------------------------------------
if [[ "${USE_OSRF_REPO}" == "true" ]]; then
  echo "==> Habilitando repo OSRF (Gazebo)"
  $SUDO bash -c "echo 'deb [arch=${ARCH}] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${CODENAME} main' \
    > /etc/apt/sources.list.d/gazebo-stable.list"
  curl -sSL http://packages.osrfoundation.org/gazebo.key \
    | $SUDO gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg
  $SUDO sed -i "s#deb \[arch=${ARCH}\] http://packages.osrfoundation.org/gazebo/ubuntu-stable#deb \[arch=${ARCH} signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg\] http://packages.osrfoundation.org/gazebo/ubuntu-stable#g" \
    /etc/apt/sources.list.d/gazebo-stable.list
  $SUDO apt-get update
  DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y ignition-fortress || true
fi

# --------------------------------------------------------------------
# 5) Gazebo Fortress integrado ao ROS (ros_gz para Humble)
# --------------------------------------------------------------------
set +e
$SUDO apt-get update
if apt-cache show "ros-${ROS_DISTRO}-ros-gz" >/dev/null 2>&1; then
  DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y "ros-${ROS_DISTRO}-ros-gz"
else
  echo "==> Metapacote ros-${ROS_DISTRO}-ros-gz não listado; tentando subpacotes..."
  PKGS="$(apt-cache search "^ros-${ROS_DISTRO}-ros-gz" | awk '{print $1}' | tr '\n' ' ')"
  if [[ -n "${PKGS}" ]]; then
    DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y ${PKGS}
  else
    echo "!! Nenhum pacote ros-gz encontrado. Verifique as fontes APT."
  fi
fi
set -e

# --------------------------------------------------------------------
# 6) Instalar Navigation2 via APT
# --------------------------------------------------------------------
DEBIAN_FRONTEND=noninteractive $SUDO apt-get install -y \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-nav2-rviz-plugins" \
  "ros-${ROS_DISTRO}-nav2-waypoint-follower" \
  "ros-${ROS_DISTRO}-slam-toolbox"

# --------------------------------------------------------------------
# 7) rosdep + build do workspace (apenas o que está no src)
# --------------------------------------------------------------------
if ! command -v rosdep >/dev/null 2>&1; then
  echo "!! rosdep não encontrado — algo falhou na instalação do ROS 2."
  exit 1
fi

set +e
$SUDO rosdep init 2>/dev/null
set -e
rosdep update

rosdep install --from-paths "${WS_PATH}/src" --ignore-src -r -y

cat <<EOF

============================================================
OK! Ambiente pronto.

Workspace: ${WS_DIR}

Para usar agora nesta sessão:
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source ${WS_DIR}/install/setup.bash

Testes rápidos:
  ros2 doctor
  ros2 pkg list | grep nav2
  ign gazebo   # (Fortress usa 'ign gazebo' em vez de 'gz sim')

Exemplos:
  ./setup_humble_gz_nav2_apt.sh                 # cria ${WS_DIR} padrão
  ./setup_humble_gz_nav2_apt.sh ~/meu_ws true   # também ativa repo OSRF
============================================================
EOF