# Workspace de Robótica Móvel – Pacotes Utilizados

Este workspace ROS 2 reúne os pacotes necessários para a realização das **Atividades Hands-On 1/2 e 3/4**, cobrindo desde a **navegação em mapas conhecidos** até a **geração de mapas por SLAM 3D e navegação ponto-a-ponto** em ambientes 3D simulados.

A organização dos pacotes reflete uma arquitetura modular, permitindo a compreensão clara da integração entre **simulação, percepção, mapeamento e navegação** no ecossistema ROS 2.

---

## 📦 Pacotes do Workspace

### `pgrsa_hunter`
Pacote responsável pela **simulação e navegação do robô Hunter**, utilizado nas atividades iniciais.

**Função:**
- Integração com o Nav2 para navegação em **mapas previamente conhecidos**.

**Utilização:**
- Hands-On **1/2**

---

### `pgrsa_scout`
Pacote responsável pela **simulação e navegação do robô Scout**, utilizado como alternativa de plataforma móvel.

**Função:**
- Integração com o Nav2 para navegação em **mapas previamente conhecidos**.

**Utilização:**
- Hands-On **1/2**

---

### `pgrsa_slam2d`
Pacote central das atividades Hands-On **3/4**, responsável pela integração entre **SLAM 3D multissensorial** e **navegação 2D (x, y, yaw)** em ambiente 3D simulado.

**Função:**
- Bringup completo do sistema robótico para SLAM e navegação;
- Execução do **SLAM 3D com RTAB-Map**;
- Geração do mapa em tempo de execução;
- Projeção do mapa 3D para navegação 2D;
- Inicialização e configuração do Nav2 para navegação ponto-a-ponto;
- Ajuste de parâmetros de SLAM e navegação.

**Principais arquivos:**
- `pgrsa_bringup_2d_slam.launch.py` – inicialização do sistema;
- `pgrsa_rtab_map.launch.py` – SLAM 3D com RTAB-Map;
- `pgrsa_nav2.launch.py` – navegação 2D com Nav2;
- `rtabmap_params.yaml` – parâmetros do SLAM;
- `2d_slam_nav2.yaml` – parâmetros de navegação.

**Utilização:**
- Hands-On **3/4**

---

## 🔗 Pacotes de Infraestrutura (ROS 2)

Além dos pacotes desenvolvidos no projeto, são utilizados pacotes oficiais do ROS 2:

- Nav2 (`nav2_bringup`, `nav2_planner`, `nav2_controller`, `nav2_bt_navigator`)
- RTAB-Map (`rtabmap_ros`)
- Ferramentas de visualização e controle (`RViz2`, `rqt_steering`)

Esses pacotes fornecem a base para planejamento, controle, visualização e mapeamento.

---

## 📊 Relação entre Pacotes e Atividades

| Atividade | Pacotes Principais | Mapeamento | Navegação |
|----------|-------------------|-----------|-----------|
| Hands-On 1/2 | `pgrsa_hunter`, `pgrsa_scout` | ❌ | Nav2 (mapa conhecido) |
| Hands-On 3/4 | `pgrsa_slam2d` | SLAM 3D (RTAB-Map) | Nav2 (mapa gerado) |

---
## 🛠️ Sobre o arquivo `ros_setup.sh`

O arquivo `ros_setup.sh` é responsável por **configurar e padronizar o ambiente ROS 2** utilizado ao longo das atividades práticas. Sua utilização garante que todas as variáveis de ambiente necessárias estejam corretamente carregadas, evitando problemas relacionados a *paths*, sobreposição de workspaces (*overlays*) e reconhecimento de pacotes.

Esse script assegura que os pacotes utilizados nos **Hands-On 1/2 e 3/4** sejam corretamente encontrados pelo ROS 2, contribuindo para a reprodutibilidade dos experimentos e reduzindo erros comuns de configuração. Essa prática é amplamente adotada em projetos acadêmicos e profissionais de robótica.


## ▶️ Como utilizar

Sempre que abrir um **novo terminal**, execute o comando abaixo antes de iniciar qualquer simulação ou navegação:

```bash
source ros_setup.sh
```

## 🔧 Compilação do Workspace com `colcon build`

O comando abaixo é utilizado para **compilar os pacotes do workspace ROS 2** de forma otimizada, gerando binários preparados para execução com melhor desempenho:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```