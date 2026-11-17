# ROBO LAB — Inteligência Artificial Aplicada à Robótica

**Software didático interativo para a Aula 7 — Aplicações Contemporâneas da Inteligência Artificial.**

Uma aplicação em Python com interface gráfica feita em **Pygame Community Edition**. Simula um robô móvel autônomo, seu sensor LiDAR, seu planejamento de rotas, o controlador de movimento e o aprendizado por reforço por comparação. Não requer robô físico, webcam, GPU, modelo pré-treinado ou conexão à internet depois da instalação das dependências.

> **Distinção importante:** a navegação principal usa **A\*** (busca informada) e uma versão didática de **Dynamic Window Approach — DWA** (otimização de trajetórias). A parte de **Q-learning** é aprendizado por reforço tabular, treinado em separado e exibido como rota alternativa, não como controlador físico em tempo real. Não há rede neural e o simulador não implementa SLAM, ROS, Isaac Sim ou navegação robótica certificada.

## Principais recursos

- **3 ambientes:** centro logístico com estantes e AGVs, resgate com destroços e corredores, e mobilidade urbana com edificações e trânsito.
- **Mapa interativo:** clique para definir o destino, adicione caixas em tempo real e remova caixas adicionadas. O planejamento é atualizado.
- **Robô com cinemática diferencial:** posição, orientação, velocidade linear e angular, acelerações máximas, rastro percorrido e indicadores de missão.
- **Sensor LiDAR simulado de 270°:** 72 feixes, alcance ajustável e retornos desenhados; o controlador aplica redução de velocidade quando um retorno frontal indica proximidade crítica.
- **Planejamento A\*:** explora uma grade do ambiente conhecido, com expansão de oito vizinhos e obstáculos inflados pelo raio do robô; exibe rota e células visitadas.
- **Controle local DWA simplificado:** avalia candidatas `(velocidade linear, velocidade angular)`, simula trajetórias por aproximadamente 0,9 s, descarta movimentos previstos que cruzariam obstáculos e pontua alinhamento, distância, velocidade e margem de segurança. **Verde** indica trajetória escolhida; **amarelo** mostra candidatas.
- **AGVs móveis:** obstáculos dinâmicos cooperativos, com inversão de sentido ao encontrar o robô. Permitem demonstrar segurança e desvio de objetos em movimento.
- **Q-learning:** treina uma política `Q(estado, ação)` de quatro direções na grade estática. Episódios, taxa de sucesso, evolução do treinamento e rota aprendida em **roxo**. Há mapa de valores aprendido para visualização.
- **Controle automático/manual:** alterne entre direção autônoma e condução pelo teclado.
- **Telemetria didática:** velocidade, rumo, distância percorrida, tempo, paradas de segurança, feixes, menor distância, quantidade de células exploradas, replanejamentos, trajetórias viáveis, velocidade e giro escolhidos, e aprendizado.
- **Capturas:** a tecla `F12` salva a tela em `capturas/`.

## Instalação — Windows, inclusive Python 3.14

1. Extraia o ZIP mantendo os arquivos juntos.
2. Clique duas vezes em **`INICIAR_WINDOWS.bat`**.
3. O iniciador procura Python 3.14, 3.13, 3.12, 3.11 ou 3.10, todos de 64 bits (prioriza o mais recente).
4. É criada a pasta `.venv` e instalada a biblioteca `pygame-ce` quando necessário.
5. O aplicativo abre automaticamente.

O arquivo `.bat` usa **ASCII sem BOM + finais de linha CRLF**, para não produzir o erro `'´╗┐@echo' não é reconhecido` no CMD. Não é necessário instalar MediaPipe ou Ultralytics: o simulador usa exclusivamente `pygame-ce` e Python padrão.

### Instalação manual

No PowerShell, dentro da pasta do aplicativo:

```powershell
py -3.14 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe robotica_interativa.py
```

O software não baixa modelos de IA. O `pygame-ce` precisa ser instalado uma única vez; depois o simulador pode funcionar offline.

**Requisitos sugeridos:** Windows 10/11; Python 3.10–3.14 (64 bits), idealmente 3.14; monitor 1366×768 ou maior (recomendado 1536×920); CPU comum. A interface é redimensionável e mantém a proporção da tela.

## Controles da interface

| Controle | Ação |
|---|---|
| **ARMAZÉM / RESGATE / CIDADE** | Trocar o cenário de simulação |
| **CLIQUE: DESTINO** | Clicar no chão livre para alterar o destino |
| **+ OBSTÁCULO** | Inserir uma caixa 40×40 clicando em espaço livre |
| **REMOVER** | Selecionar uma caixa inserida e excluí-la |
| **Botão direito do mouse** | Remover uma caixa inserida próxima ao clique |
| **LiDAR** | Exibir/ocultar feixes e ecos |
| **ROTA A\*** | Exibir/ocultar caminho calculado pelo A\* |
| **BUSCA** | Mostrar células expandidas pelo A\* |
| **ROTA Q** | Exibir/ocultar política Q-learning aprendida |
| **MAPA Q** | Mostrar valores de estados Q em um mapa colorido |
| **AGVs** | Exibir/ocultar agentes móveis |
| **Treinar IA** | Executar aprendizado por reforço; pode pausar ou reiniciar |
| **− / +** | Ajustar velocidade máxima, LiDAR e margem de segurança |
| **AUTÔNOMO / MANUAL** | Alternar modo de movimento |
| **W/A/S/D ou setas** | Dirigir apenas no modo MANUAL |
| **1, 2, 3** | Trocar cenários |
| **Espaço** | Pausar/retomar |
| **R** | Reiniciar cenário |
| **L / P / H** | Alternar LiDAR / rota A\* / células exploradas |
| **T ou Q** | Iniciar/pausar treinamento Q-learning |
| **M** | Alternar entre manual e automático |
| **F12** | Salvar captura PNG |
| **Ajuda / Esc** | Abrir/fechar guia didático |

## Aula demonstrativa (15–20 minutos)

1. **Percepção (3 min):** abra **ARMAZÉM**, ative LiDAR e mostre como os 72 raios retornam distâncias. Ajuste o alcance e aproxime o robô das estantes.
2. **Planejamento global (3 min):** ative **ROTA A\*** e **BUSCA**. Explique a grade, a heurística e o custo de movimento. Clique em novo destino e compare a região explorada.
3. **Planejamento local (3 min):** observe as trajetórias candidatas amarelas e a escolhida verde (DWA). Habilite os AGVs e discuta segurança, aceleração e mudança de rota.
4. **Interação (3 min):** clique em **+ OBSTÁCULO** e coloque uma caixa adiante do robô, sem sobrepor o robô, estantes ou meta. Observe o novo plano. Remova a caixa em seguida.
5. **Aprendizado por reforço (4 min):** use **REINICIAR** e clique em **TREINAR IA**; observe o gráfico de sucesso da política Q. Ative **ROTA Q** e **MAPA Q**. Compare a rota aprendida com o A\* — elas não precisam ser idênticas.
6. **Autonomia x intervenção (3 min):** alterne **MANUAL** e use `WASD`, depois retome **AUTÔNOMO**. Discuta por que perceber, planejar e agir são etapas separadas.

## Como a IA funciona

### A\*: planejamento sobre um mapa conhecido

A busca A\* usa:

\[
f(n) = g(n) + h(n)
\]

- `g(n)` é o custo acumulado até o estado `n`;
- `h(n)` é a estimativa da distância euclidiana restante até a meta;
- a grade tem 25 unidades por célula;
- o raio do robô e uma margem expandem os obstáculos na grade para evitar caminhos muito estreitos.

O caminho bruto é suavizado por testes de visibilidade direta, mantendo uma margem geométrica. Os obstáculos móveis são tratados prioritariamente pelo controlador local para evitar oscilações no planejamento global.

### DWA didático: decisão local

A cada aproximadamente 0,18 s, gera-se um conjunto de velocidades lineares e angulares candidatas. Para cada uma, o robô estima sua trajetória de até 0,90 s no futuro. Trajetórias que cruzariam obstáculos dentro da margem configurada são eliminadas; as demais são pontuadas por proximidade da próxima referência, alinhamento, velocidade e espaço livre. A melhor é executada, respeitando limites de aceleração e velocidade angular.

**Não se trata da implementação industrial completa de DWA**, mas de uma versão explicável, apropriada para demonstração. A dinâmica de AGVs é idealizada, com velocidade prevista a partir de sua trajetória geométrica; não existe rastreamento probabilístico nem fusão sensorial real.

### LiDAR simulado e freio emergencial

Os feixes são calculados por interseções de raios com círculos, retângulos e bordas. A menor distância no setor frontal pode limitar a velocidade em uma aproximação crítica. **O mapa estático é previamente conhecido**: os retornos do LiDAR não constroem um mapa por SLAM. Por isso, a demonstração separa percepção de uma etapa de mapeamento real.

### Q-learning: aprendizagem com recompensa

O aprendiz Q usa quatro ações discretas em um mapa de células estáticas. A atualização é:

\[
Q(s,a) \leftarrow Q(s,a)+ \alpha[r+\gamma \max_{a'}Q(s',a')-Q(s,a)]
\]

- `s`: célula ocupada pelo agente de treinamento;
- `a`: deslocamento para uma das quatro direções;
- `r`: recompensa de atingir a meta, penalidade por colisão e custo de passo, com incentivo de progresso;
- `α = 0,36`: taxa de aprendizado;
- `γ = 0,96`: desconto de recompensas futuras;
- seleção de ações `ε`-gulosa, com exploração decrescente;
- até **2.400 episódios** por treinamento, com limite de 150 passos por episódio.

A política aprendida é **uma visualização comparativa**, e não comanda a cinemática contínua do robô; os AGVs móveis não entram no ambiente de treinamento. Isso é intencional, para demonstrar limitações e diferenças entre busca clássica e aprendizado por reforço.

## Estrutura do projeto

```text
Robotica_IA_Interativa_Aula_7/
├── INICIAR_WINDOWS.bat       # inicializador sem BOM
├── README.md                 # este manual
├── requirements.txt          # pygame-ce
├── robotica_interativa.py    # UI, desenho e interações
├── navegacao.py              # geometria, LiDAR, A*, DWA, Q-learning, simulação
├── testes/
│   └── test_navegacao.py     # testes automatizados sem pygame
└── .gitignore
```

A pasta `.venv` será criada na primeira execução; `capturas/` aparece quando você salvar uma imagem. Nenhum desses conteúdos é necessário para publicar o projeto no GitHub.

## Testes

Com Python instalado (não requer pygame):

```powershell
py -3.14 -m unittest discover -s testes -v
```

A bateria verifica geometria de obstáculos, detecção LiDAR, A\*, não atravessar cantos bloqueados, suavização de caminhos, pontuação DWA, convergência Q-learning, estados de pausa, controle manual e **conclusão autônoma dos três cenários sem sobreposição física com os obstáculos simulados**.

## Limitações e boas práticas

- A velocidade e distância são expressas em **unidades simuladas**, não em m/s ou metros medidos.
- O algoritmo pode falhar ou ficar sem rota em mapas editados que sejam inviáveis; isso também é resultado didático válido.
- Não há aprendizado da política durante a navegação contínua: o Q-learning é treinado em grade estática e exibido para comparação.
- As coordenadas de obstáculos móveis são conhecidas pela simulação, idealizando o rastreamento de objetos; o LiDAR apresentado não recria toda uma percepção robótica real.
- O modelo é uma simulação educacional: não deve ser transferido diretamente para operação de robôs reais nem utilizado para segurança de pessoas ou equipamentos.
- A interface e o Pygame devem ser testados no computador do usuário. Neste ambiente de criação, o `pygame-ce` não estava instalado e o acesso ao instalador PyPI não estava disponível; a validação executada concentrou-se na lógica do simulador, testes unitários e na verificação sintática do código.

## Bibliotecas

- **pygame-ce:** https://pypi.org/project/pygame-ce/

O simulador não utiliza imagens nem conteúdo de terceiros; todos os elementos da interface são desenhados por código.


## Correcao da interface (16/11/2026)

Corrigido o erro `AttributeError: 'tuple' object has no attribute 'centerx'`
na criacao dos botoes. A funcao `create_button` agora normaliza
os argumentos para `pygame.Rect`, tanto para a renderizacao quanto
para a identificacao de cliques. A atualizacao nao altera os cenarios
nem os algoritmos de navegacao.

Para atualizar uma instalacao existente, substitua o arquivo
`robotica_interativa.py` ou extraia novamente o pacote. A pasta `.venv`
pode ser mantida.
