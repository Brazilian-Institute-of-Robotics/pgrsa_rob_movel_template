# Atenção às instruções abaixo.

Olá, estudante!

Este repositório é um **template** para o desenvolvimento da atividade do Roteiro hands on 1 e 2 e Roteiro hands on 3 e 4 da Unidade Curricular **Robótica Móvel e Autônoma**. 

**Há duas formas de criar o workspace ROS2:**
 - **Automática usando o script**
 - **Manual diretamente no terminal.**
---
--- 
# Setup ROS 2 Humble + Gazebo + Nav2 (script) Automático

> **Objetivo:** executar o script de instalação e depois dar o source no workspace manualmente.  
> **Compatibilidade:** Ubuntu 22.04 (Jammy).

---

## 📦 O que o script realiza:

- Configura o repositório oficial do ROS 2 (`ros-apt-source`);
- Instala **ROS 2 Humble (desktop)**, `colcon`, `rosdep`, `vcstool` e ferramentas;
- (Opcional) habilita repositório **OSRF** e instala **Ignition Fortress**;
- Instala **ros-gz** (integração Gazebo ↔ ROS);
- Instala **Navigation2**;
- Clona `ugv_gazebo_sim` (branch `humble`).
- Clona o repositório `PGRSA_ROB_MOVEL_TEMPLATE`

---

## ✅ Pré-requisitos

- Ubuntu 22.04 (Jammy);
- Usuário com `sudo`.

---

## ▶️ Execute o script

```bash
chmod +x ./setup_humble_gz_nav2_apt.sh
./setup_humble_gz_nav2_apt.sh
```

Argumentos opcionais:

| Parâmetro | Padrão     | Exemplo                       | Descrição                                   |
|----------:|------------|-------------------------------|---------------------------------------------|
| `<WS>`    | `pgrsa_ws` | `~/ros_ws`                    | Caminho/nome do workspace                   |
| `[OSRF]`  | `false`    | `true`                        | Ativa repo OSRF (Ignition Fortress)         |

Exemplos:
```bash
./setup_humble_gz_nav2_apt.sh                 # cria ./pgrsa_ws ao lado do script
./setup_humble_gz_nav2_apt.sh ~/ros_ws true   # cria ~/ros_ws e ativa OSRF
```

> O script exibirá o **caminho absoluto** do workspace ao final (ex.: `/caminho/para/pgrsa_ws`).

---
## Na raíz do diretório execute o comando para inicializar o submódulo:

```
git submodule update --init --recursive
```

## 🧭 Após executar o script em **um novo terminal** utilize os comandos a seguir:

1. **Ir para o workspace** (use o caminho mostrado pelo script):
   ```bash
   cd /caminho/para/pgrsa_ws
   ```

2. **Carregar o ambiente do ROS 2 (do `/opt`)**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Construir com colcon**:
   ```bash
   colcon build --symlink-install
   ```

4. **Carregar o overlay do workspace**:
   ```bash
   source install/setup.bash
   ```

*(Opcional) Gravar no `~/.bashrc`:*
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /caminho/para/pgrsa_ws/install/setup.bash' >> ~/.bashrc
```

---

## 🔎 Testes rápidos para verificar a instalação (opcional)

```bash
ros2 doctor
ros2 pkg list | grep -E 'nav2|ros_gz|gazebo|rviz|robot_state_publisher'
```

Se instalou o Gazebo Fortress:
```bash
ign gazebo
```

---

## 🛟 Solução de problemas

**Erro `unbound variable` ao dar `source` (shell com `set -u`/`nounset`):**

**Opção A** — desativar `-u` só durante o `source`:
```bash
set +u
source /opt/ros/humble/setup.bash
set -u
```

**Opção B** — definir default e então `source`:
```bash
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-0}
source /opt/ros/humble/setup.bash
```

---

## 📁 Estrutura esperada do workspace

```
pgrsa_ws/
├─ src/
│  └─ ugv_gazebo_sim/   (branch humble)
│  └─ pgrsa_hunter/   
│  └─ pgrsa_scout/
|  └─ pgrsa_2d_slam/    
├─ build/               (gerado)
├─ install/             (gerado)
└─ log/                 (gerado)
```

---


# Instalação Manual — ROS 2 Humble + Gazebo + Nav2

> **Objetivo:** Passo a passo para instalar **sem** usar script. Compatível com **Ubuntu 22.04 (Jammy)**.

---

## 1) Pré-requisitos

```bash
sudo apt-get update
sudo apt-get install -y   software-properties-common curl gnupg lsb-release ca-certificates   git python3-pip python3-venv   build-essential cmake
sudo add-apt-repository -y universe
```

---

## 2) Adicione a fonte APT oficial do ROS 2

```bash
# Obtenha a última versão do ros-apt-source e instale
ver="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest        | grep -F tag_name | awk -F\" '{print $4}')"
curl -L -o /tmp/ros2-apt-source.deb   "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.jammy_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt-get update
sudo apt-get -y upgrade
```

---

## 3) Instale o ROS 2 Humble (desktop) + ferramentas

```bash
sudo apt-get install -y   ros-humble-desktop   python3-colcon-common-extensions   python3-rosdep   python3-vcstool   ros-dev-tools
```

Inicialize o **rosdep** (uma vez por máquina):

```bash
sudo rosdep init || true
rosdep update
```

---

## 4) (Opcional) Instale Gazebo (Ignition Fortress) + ros-gz

```bash
# Repositório OSRF (opcional) para Fortress
ARCH="$(dpkg --print-architecture)"
echo "deb [arch=${ARCH}] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main"   | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
curl -sSL http://packages.osrfoundation.org/gazebo.key   | sudo gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg
sudo sed -i "s#deb \[arch=${ARCH}\] http://packages.osrfoundation.org/gazebo/ubuntu-stable#deb \[arch=${ARCH} signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg\] http://packages.osrfoundation.org/gazebo/ubuntu-stable#g"   /etc/apt/sources.list.d/gazebo-stable.list

sudo apt-get update
sudo apt-get install -y ignition-fortress || true

# Integração ROS <-> Gazebo (ros_gz)
sudo apt-get install -y ros-humble-ros-gz ||   sudo apt-get install -y $(apt-cache search '^ros-humble-ros-gz' | awk '{print $1}')
```

---

## 5) Instale Navigation2

```bash
sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

---

## 6)Na raiz do diretório execute o comando para inicializar o submódulo:

```
git submodule update --init --recursive
```

Instale as dependências do workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 7) Build e configuração do ambiente

Abra um **novo terminal** e execute:

```bash
source /opt/ros/humble/setup.bash
cd ~/pgrsa_ws
colcon build --symlink-install
source install/setup.bash
```

*(Opcional) Gravar no `~/.bashrc`:*

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/pgrsa_ws/install/setup.bash' >> ~/.bashrc
```

---
