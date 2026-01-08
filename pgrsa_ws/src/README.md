# Entrega do Vídeo de Navegação (Nav2 em mapa conhecido)

**Objetivo**  
Realizar a navegação segura em um ambiente já mapeado, ajustando os parâmetros de controladores e planejadores do Nav2 conforme o tipo de robô.  
A entrega consiste em **1 vídeo** (ambos os robôs no mesmo vídeo) **com os links colados neste README**.

---

---
**Lista de Aluno(s)**

Nome dos aluno(s):
- nome1
- nome2
- ...

---

## ✅ O que entregar

- **Vídeo(s) no YouTube** mostrando:
  - Simulação em funcionamento;
  - Definição dos pontos de navegação no RViz2 via `2D_Goal_Pose` ou `Waypoint_Follower`;
  - Percurso completo **O → A → B → C → O**, sem colisões e com comportamento estável.
- **Links do(s) vídeo(s)** nas seções abaixo:
  - **Ackermann:** _cole o link aqui_
  - **Diferencial:** _cole o link aqui_

> **Importante:** o vídeo pode ser **único** (com os dois robôs) 
> **Nome sugerido no YouTube:** `PGRSA Robótica Móvel — Navegação Nav2 — <EQUIPE> — <Ackermann|Diferencial>`.

---


Entrega do Vídeo de SLAM 3D e Navegação (RTAB-Map + Nav2)
Objetivo

Realizar a navegação ponto-a-ponto de um robô móvel em ambiente 3D simulado, utilizando SLAM 3D multissensorial para geração do mapa em tempo de execução e navegação 2D (x, y, yaw) com o Nav2.
A atividade envolve a exploração manual do ambiente, a construção do mapa, o ajuste de parâmetros e a validação da navegação autônoma.

A entrega consiste em 1 vídeo publicado no YouTube, com o link inserido neste README.

Lista de Aluno(s)

Nome do(s) aluno(s):

nome1

nome2

...

✅ O que entregar

Vídeo no YouTube mostrando claramente:

Ambiente de simulação em funcionamento (Gazebo);

Execução do SLAM 3D com RTAB-Map durante a exploração do ambiente;

Mapa gerado em tempo de execução visualizado no RViz2;

Navegação autônoma utilizando o mapa gerado;

Definição de metas no RViz2 via 2D Goal Pose (mínimo de duas metas distintas);

Execução completa das trajetórias sem colisões e com comportamento estável.

Durante a navegação, devem estar visíveis no RViz2:

Mapa (/map);

Modelo do robô (robot_description);

Caminho global planejado (planner);

Rastreamento pelo controlador local;

Costmaps global e local.

🔗 Link do Vídeo

SLAM 3D + Navegação 2D:
cole o link do vídeo aqui

Importante:

O vídeo deve evidenciar tanto a fase de mapeamento quanto a de navegação.

Recomenda-se manter Gazebo e RViz2 lado a lado, com o cursor visível ao definir as metas.

Nome sugerido no YouTube:
PGRSA Robótica Móvel — SLAM 3D e Navegação — <EQUIPE>

📝 Observações

Ajustes de parâmetros realizados nos arquivos de configuração (RTAB-Map e Nav2) devem refletir melhoria no mapa e/ou na navegação.

Alterações relevantes devem estar versionadas no repositório e podem ser comentadas durante o vídeo ou no histórico de commits.