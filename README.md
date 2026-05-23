# Starting
## Setup

1. Create and activate a virtual environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

2. Install dependencies:

```powershell
python -m pip install -U pip
python -m pip install -e .
```

3. Run tests:

```powershell
python -m pytest
```

# Estructura del código v1
src/
  main.py        — bucle principal, máquina de estados, logs
  robot.py       — clase Robot (encapsula handle, motores, sensores, control)
  scene.py       — funciones de escena (posiciones, vectores, condición de éxito)
  strategy.py    — lógica de roles, fases, coordinación

# Estructura del código v2
src/
    main.py      — bucle principal, máquina de estados global
    robot.py     — clase Robot (sin cambios relevantes)
    drone.py     — clase Drone (waypoints, detección geométrica, control de target)
    scene.py     — geometría, vectores, condición de éxito
    strategy.py  — coordinación entre los tres agentes

# Diseño conceptual
Geometría de la escena

Tablero útil: ~±2.4m en x e y (5x5m con paredes externas)
RobotA en (-1.75, -1.425), RobotB en (-1.75, 1.85), ambos mirando hacia -X
Caja en (0,0), target en (1.5, 0.5)
Vector de empuje necesario: (1.5, 0.5) normalizado ≈ (0.949, 0.316)
El robot necesita posicionarse en el lado opuesto al target respecto a la caja, es decir, en dirección (-0.949, -0.316) desde la caja

#Roles
RobotA = Pusher (empujador principal)
Se posiciona detrás de la caja en la dirección opuesta al target y empuja. Es el líder en este escenario porque está más cerca de la posición de empuje óptima (está abajo-izquierda, el punto de empuje también es abajo-izquierda).
RobotB = Corrector lateral
Se posiciona perpendicular a la trayectoria. Solo interviene si la caja se desvía lateralmente más de un umbral. Si no hay desviación, sigue la caja a distancia sin interferir. Esto evita el problema de fuerzas canceladas.

# Fases de ejecución v1
FASE 0 — Planificación
  Calcular vector de empuje (caja → target)
  Asignar roles (en este escenario fijo, en futuro por distancia)
  
FASE 1 — Posicionamiento
  A y B se mueven a sus posiciones de standoff simultáneamente
  A: detrás de la caja a ~0.5m (radio robot ~0.3m + margen)
  B: lateral a la trayectoria, mismo plano que la caja, a ~0.5m
  Condición de paso: ambos en posición ± 0.1m

FASE 2 — Empuje coordinado
  A empuja a velocidad reducida (permite que B reaccione)
  B monitoriza desviación lateral de la caja cada ciclo:
    si desviación > 0.1m → B corrige empujando lateralmente
    si desviación < 0.05m → B solo sigue (velocidad mínima)
  Condición de éxito: cualquier punto de la caja toca target
    = distancia(centro_caja, target) < 0.1m (radio_caja = 0.1m)

FASE 3 — Stop
  Ambos motores a 0
  Log de tiempo total y desviación acumulada

# Fases de ejecución v2

FASE 0 — PATRULLA
  qua vuela waypoints sobre el tablero
  roba y robo se mueven en sectores asignados (mitad izquierda cada uno)
  Ninguno conoce la posición de box

FASE 1 — DETECCIÓN
  qua entra en radio 0.8m sobre la caja → la detecta
  Comunica coordenadas a roba y robo (variable compartida en Python)
  qua se queda sobrevolando la caja como referencia visual

FASE 2 — CONVERGENCIA
  roba y robo navegan hacia sus posiciones de empuje
  Lado a lado detrás de la caja, separados ~0.45m
  Ambos alineados hacia /target

FASE 3 — EMPUJE CONJUNTO
  Velocidades sincronizadas
  Con 20kg de caja y dos robots empujando, la cooperación es física y necesaria
  Condición de éxito: caja dentro de 0.2m del target