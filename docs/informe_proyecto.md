# Coordinación cooperativa de robots móviles mediante descomposición de fuerzas: empuje cooperativo de una carga hasta un punto de extracción

---

## Resumen

Este proyecto aborda el problema de coordinación multi-robot para el transporte cooperativo de una carga hasta un punto de extracción predefinido. Se propone e implementa un sistema de planificación centralizado basado en descomposición de fuerzas mediante mínimos cuadrados con restricción de no negatividad (NNLS, *Non-Negative Least Squares*), que asigna a cada robot una cara de contacto y una magnitud de fuerza tales que la resultante vectorial coincide con la dirección óptima de empuje. El sistema integra un navegador basado en *waypoints* seguros, una máquina de estados por robot (NAVIGATE → PUSH → BACKOFF) y múltiples mecanismos de recuperación ante situaciones de bloqueo. Los resultados experimentales en CoppeliaSim demuestran que la configuración de dos robots alcanza el objetivo en los cinco escenarios de dirección de *rally* evaluados, con un mínimo de 1 327 pasos en el caso más favorable.

---

## 1. Introducción

El transporte colaborativo de objetos pesados mediante robots móviles constituye un problema fundamental en robótica multi-agente. Cuando la masa de la carga supera la capacidad de empuje de un único robot, o cuando la geometría de la escena lo requiere, es necesario que varios agentes cooperen de forma coordinada. El reto no es trivial: cada robot ejerce una fuerza en una dirección dictada por su posición relativa a la carga, y la combinación de ambas fuerzas debe resultar en el vector deseado sin introducir par rotacional que desvíe la trayectoria.

El objetivo del proyecto es diseñar e implementar un sistema que permita a dos robots Pioneer P3DX transportar una carga cúbica de 20 kg hasta un punto de extracción (*rally point*) en una arena de 5 × 5 m, utilizando únicamente la posición terrestre obtenida a través de la API del simulador. El sistema debe ser robusto ante distintas orientaciones del punto de extracción y recuperarse de situaciones de bloqueo sin intervención externa.

---

## 2. Entorno de simulación

### 2.1 Plataforma

El desarrollo y la validación se realizan íntegramente en **CoppeliaSim**, simulador de robótica que incorpora el motor de física **Bullet v2.78**. La comunicación entre el código de control en Python y el simulador se establece mediante la **ZMQ Remote API** (`coppeliasim-zmqremoteapi-client`), que expone las primitivas de lectura de sensores, control de motores y consulta de posición en tiempo real.

El bucle de control opera en modo **síncrono** (`sim.setStepping(True)` + `sim.step()`): cada iteración del código Python avanza exactamente un paso de simulación, garantizando reproducibilidad y eliminando condiciones de carrera entre el control y la física.

### 2.2 Escena

| Objeto | Posición inicial | Propiedades |
|--------|------------------|-------------|
| `/p3dx_1` | (−1.725, −1.475) | 9 kg, respondable, dinámico |
| `/p3dx_2` | (−1.750, 1.850) | 9 kg, respondable, dinámico |
| `/payload` | (0, 0) | 20 kg, respondable, dinámico |
| `/rally_point` | variable | solo posición conocida |
| `/Floor` | — | 500 kg, no respondable, estático |

La fricción tanto del suelo como de la carga tiene valor 1.0. La carga pesa más del doble que cada robot; en consecuencia, un único robot puede moverla, pero con rendimiento significativamente inferior al cooperativo.

### 2.3 Robots

Los agentes son modelos **Pioneer P3DX**, plataforma diferencial ampliamente utilizada en investigación. Sus características relevantes para el sistema son:

- Ancho de chasis: 0.415 m (mitad: 0.208 m, redondeado a 0.19 m con margen).
- 16 sensores ultrasónicos. Los índices 3 y 4 (frontales próximos, ±10°) se emplean para la detección de contacto y la corrección de centrado lateral durante el empuje.
- Rango máximo de sensores: 0.5 m (Lua), extendido hasta 5 m mediante lectura directa vía API ZMQ.

---

## 3. Fundamentos teóricos

### 3.1 Formulación del empuje cooperativo

El requisito fundamental de diseño es que las fuerzas de ambos robots compongan el vector de fuerza deseado:

$$\mathbf{F}_{des} = f_0 \, \hat{n}_0 + f_1 \, \hat{n}_1$$

donde:

- $\mathbf{F}_{des}$ es el vector unitario desde la posición de la carga hasta el punto de extracción.
- $\hat{n}_0, \hat{n}_1$ son los vectores unitarios de empuje de cada robot (dirección desde la posición de aproximación hacia el centro de la carga).
- $f_0, f_1 \geq 0$ son los escalares de fuerza de cada robot. La restricción de no negatividad refleja la imposibilidad física de que un robot tire de la carga.

### 3.2 Mínimos cuadrados con no negatividad (NNLS)

Dado que en el caso general no existe una combinación exacta $f_0, f_1$ que satisfaga la igualdad anterior, el problema se formula como una minimización de residuo cuadrático sujeta a restricción de no negatividad:

$$\min_{f_0, f_1 \geq 0} \left\| f_0 \hat{n}_0 + f_1 \hat{n}_1 - \mathbf{F}_{des} \right\|^2$$

Este es un problema NNLS de 2 variables, resoluble en forma cerrada. La solución óptima pertenece a uno de cuatro casos candidatos:

1. Solución interior sin restricciones: $\mathbf{G} \mathbf{f} = \mathbf{b}$, donde $G_{ij} = \hat{n}_i \cdot \hat{n}_j$ y $b_i = \hat{n}_i \cdot \mathbf{F}_{des}$. Solo válida si $f_0, f_1 \geq 0$.
2. Frontera $f_0 = 0$: $f_1 = \max(0, \hat{n}_1 \cdot \mathbf{F}_{des})$.
3. Frontera $f_1 = 0$: $f_0 = \max(0, \hat{n}_0 \cdot \mathbf{F}_{des})$.
4. Origen: $f_0 = f_1 = 0$ (descartado si ambos son nulos, ya que no produce movimiento útil).

El mínimo entre los candidatos factibles es la solución exacta. Esta implementación no requiere ninguna biblioteca externa de optimización.

### 3.3 Marco de 8 direcciones

Las posiciones de aproximación se discretizan en 8 direcciones cardinales e intercardinales: N, S, E, W, NE, NW, SE, SW. Para cada dirección $k$, la posición de aproximación es:

$$\mathbf{p}_{app}(k) = \mathbf{p}_{payload} + d_{push} \cdot \hat{o}(k)$$

donde $d_{push} = 0.489$ m y $\hat{o}(k)$ es el vector opuesto a la cara correspondiente (e.g., para cara N: $\hat{o} = (0, -1)$, el robot se acerca desde el sur).

La dirección de empuje no es fija sino dinámica:

$$\hat{n}(k) = \frac{\mathbf{p}_{payload} - \mathbf{p}_{app}(k)}{\|\mathbf{p}_{payload} - \mathbf{p}_{app}(k)\|}$$

Esto garantiza que la dirección de empuje apunta siempre al centro de la carga independientemente de su posición actual, eliminando errores de alineación por desplazamiento acumulado.

### 3.4 Media móvil exponencial (EMA) para detección de estancamiento

El progreso de la carga en cada paso se calcula como la proyección del desplazamiento sobre la dirección deseada:

$$\text{progress}_t = \Delta \mathbf{p}_{payload} \cdot \mathbf{F}_{des}$$

La media móvil exponencial suaviza las fluctuaciones puntuales:

$$\text{EMA}_t = \alpha \cdot \text{progress}_t + (1 - \alpha) \cdot \text{EMA}_{t-1}$$

con $\alpha = 0.1$. Si la EMA permanece por debajo del umbral mínimo durante `STALL_STEPS` pasos consecutivos, se declara estancamiento y se lanza una rutina de recuperación. La EMA solo se actualiza cuando ambos robots están simultáneamente en fase PUSH, evitando que períodos de navegación individual contaminen la estimación de progreso cooperativo.

### 3.5 Corrección de centrado lateral

Durante la fase de empuje, los sensores frontales d3 y d4 miden la distancia de la cara de la carga al lado izquierdo y derecho del robot respectivamente. Si ambos están en rango de detección ($< d_{detect} = 0.80$ m), se aplica una corrección lateral:

$$\hat{n}_{corr} = \text{normalize}(\hat{n} + K_{center} \cdot (d_4 - d_3) \cdot \hat{n}^{\perp})$$

donde $\hat{n}^{\perp} = (-n_y, n_x)$ es el vector perpendicular a la dirección de empuje y $K_{center} = 3.0$. Esta corrección mantiene al robot centrado sobre la cara asignada, previniendo que el contacto se desplace hacia una esquina de la carga.

---

## 4. Arquitectura del sistema

### 4.1 Visión general de módulos

```
src/
  planner.py   — ContactPlanner: NNLS, 8 direcciones, waypoints, asignación pegajosa
  strategy.py  — Strategy: FSM de misión + _RobotAgent por robot
  robot.py     — Robot: motores, sensores, drive_to, get_yaw
  scene.py     — Geometría de escena: dist2d, push_vector, payload_reached_rally_point
  grid.py      — Mapa de ocupación 2D compartido (50×50 celdas, 0.1 m/celda)
  main.py      — Punto de entrada: inicialización, bucle de control principal
```

### 4.2 ContactPlanner — planificador de fuerza

El planificador recibe en cada llamada las posiciones de ambos robots, la posición de la carga y el punto de extracción. Su algoritmo es:

1. Calcular $\mathbf{F}_{des}$.
2. Enumerar todos los pares no ordenados de direcciones $\{k_0, k_1\}$ (28 pares de 8 direcciones tomadas de 2 en 2).
3. Para cada par, evaluar ambas asignaciones de robot $(R_0 \to k_0, R_1 \to k_1)$ y $(R_0 \to k_1, R_1 \to k_0)$:
   - **Filtro de separación**: rechazar si la distancia entre posiciones de aproximación es inferior a 0.52 m (ancho Pioneer 0.415 m + margen 0.10 m).
   - **Filtro geométrico**: rechazar si la posición de aproximación está en el lado del *rally* respecto a la carga (el robot empujaría en sentido contrario al objetivo).
   - Resolver NNLS para obtener $f_0, f_1$.
   - Puntuar la asignación.
4. Seleccionar la asignación de menor puntuación.
5. Generar *waypoints* seguros para cada robot.
6. Bloquear la asignación (*sticky lock*).

### 4.3 Función de puntuación

La puntuación de cada asignación candidata combina múltiples términos:

$$\text{score} = r + w_p \cdot p_p + w_a \cdot p_a + w_b \cdot b + w_t \cdot \tau + w_n \cdot c_n - \text{lock}$$

| Término | Símbolo | Descripción | Peso |
|---------|---------|-------------|------|
| Residuo | $r$ | $\|\mathbf{F}_{res} - \mathbf{F}_{des}\|$ | 1.0 |
| Penalización de progreso | $p_p$ | $1 - \cos(\angle(\mathbf{F}_{res}, \mathbf{F}_{des}))$ | 0.50 |
| Penalización de alineación | $p_a$ | Suma de desalineaciones individuales | 0.30 |
| Balance | $b$ | $|f_0 - f_1|$ | 0.20 |
| Par rotacional | $\tau$ | $|f_0 (\mathbf{r}_0 \times \hat{n}_0) + f_1 (\mathbf{r}_1 \times \hat{n}_1)|$ | 0.10 |
| Coste de navegación | $c_n$ | Distancia total normalizada (÷ 3 m) | 0.30 |
| Bonificación de bloqueo | lock | Continuidad de asignación previa | −0.10 por robot |

El peso de coste de navegación fue incrementado de 0.08 a 0.30 durante el desarrollo, como se describe en §6.

### 4.4 _RobotAgent — máquina de estados por robot

Cada robot ejecuta de forma independiente una máquina de estados de tres fases:

```
NAVIGATE → PUSH → BACKOFF
    ↑                  |
    └──────────────────┘
```

**NAVIGATE**: El robot sigue los *waypoints* seguros hasta alcanzar la posición de aproximación (umbral: 0.05 m). En cada replanificación (cada `REPLAN_EVERY = 10` pasos), si la cara asignada no cambia y el robot está en NAVIGATE, la posición de aproximación y los *waypoints* se congelan; esto evita que el robot persiga indefinidamente un objetivo que se desplaza con la carga. Si la aproximación no se alcanza en `NAV_TIMEOUT = 400` pasos, se activa un reinicio.

**PUSH**: El robot aplica el vector de empuje calculado por NNLS con corrección de centrado. Un contador de estancamiento incremental (`_contact_stall_ctr`) solo se activa cuando el robot está en contacto físico *y* ambos robots empujan simultáneamente; el empuje en solitario contra una carga pesada es esperado y no constituye un bloqueo.

**BACKOFF**: Tras `CONTACT_STALL_THR = 30` ticks consecutivos de estancamiento, el robot retrocede a velocidad reducida (50 % de la velocidad de aproximación) durante `BACKOFF_STEPS = 25` pasos y regresa a NAVIGATE.

### 4.5 Strategy — orquestación de la misión

La clase `Strategy` coordina las fases de misión mediante una FSM de alto nivel:

| Fase | Descripción |
|------|-------------|
| EXPLORING | Los robots se dirigen al *rally point*; un sensor detecta la carga en el camino |
| ACTIVE | Se activa `ContactPlanner`; los agentes ejecutan NAVIGATE→PUSH→BACKOFF |
| SUCCESS | La carga ha alcanzado el *rally point* (umbral: 0.30 m) |

En ACTIVE, además de la replanificación periódica, la función `_plan_viable()` comprueba si la tasa de progreso actual (EMA) es suficiente para alcanzar el objetivo dentro del presupuesto de pasos restante. Si no lo es, se fuerza una replanificación inmediata.

La adaptación de velocidad ajusta dinámicamente el factor de escala de empuje:

$$v_{scale} = \text{clamp}\!\left(0.70,\ 1.30,\ \frac{d_{rally} / (T_{max} - t)}{\text{EMA}_{progress}}\right)$$

---

## 5. Implementación

### 5.1 Generación de waypoints seguros

Cuando el camino directo entre la posición actual del robot y la posición de aproximación intersecta la carga (distancia mínima al centro < 0.45 m, derivada como suma de Minkowski: 0.25 + 0.19 + 0.01 m), se genera un desvío:

- **Primer intento**: punto intermedio único muestreado en 8 direcciones alrededor de la carga a radios crecientes (1.2×, 1.5×, 2.0×, 3.0× del margen de clearance). Se selecciona el camino más corto válido.
- **Segundo intento (respaldo)**: ruta en L con dos *waypoints* en columna vertical, útil cuando ambos segmentos del primer intento son bloqueados simultáneamente.

El algoritmo también evita la posición de aproximación del otro robot como obstáculo adicional.

### 5.2 Histéresis de asignación

Para evitar oscilaciones entre configuraciones equivalentes, se implementa una histéresis doble:

- **Bonificación de bloqueo** (−0.10 por robot): favorece la asignación previa en la puntuación.
- **Umbral PLAN_EPSILON = 0.05**: la asignación actual solo se reemplaza si la nueva candidata la supera por más de este margen (sumado a la bonificación de bloqueo: ventaja efectiva necesaria ≥ 0.25 en puntuación bruta).

### 5.3 Sincronización de sensores

Cada robot lee sus propios sensores d3/d4 de forma independiente en cada paso. No existe sincronización ni barrera de espera entre agentes: cada robot actúa conforme a su estado y sus observaciones propias en cada tick del bucle de control.

---

## 6. Resultados experimentales

### 6.1 Línea base — robot individual

Como referencia de rendimiento de un único agente:

| Métrica | Valor |
|---------|-------|
| Pasos hasta objetivo | 826 |
| Repositorios | 0 |
| Distancia final al *rally* | 0.3293 m |
| Reproducibilidad | Determinista |

### 6.2 Evolución del planificador cooperativo

| Configuración | Pasos | Resets por estancamiento | Ratio $f_0/f_1$ |
|---------------|-------|--------------------------|-----------------|
| Cardinal N+E (sin NNLS) | 4987 | 0 | 0.33 |
| Cardinal N+E (con NNLS) | 4987 | 0 | 0.33 |
| 8 direcciones NE+SE | **1466** | 0 | 0.50 → 0.17 |

La transición de 4 a 8 direcciones supone una reducción del 70.6 % en número de pasos. El par NE+SE para rally en dirección noreste ofrece residuo nulo (reconstrucción perfecta de $\mathbf{F}_{des}$) con separación de aproximación de 0.692 m.

### 6.3 Benchmark multi-escenario

Tras la aplicación de las correcciones descritas en §7, el sistema supera satisfactoriamente los cinco escenarios canónicos:

| Escenario | Resultado | Pasos | Resets |
|-----------|-----------|-------|--------|
| NE (línea base) | ÉXITO ✓ | 1 327 | 0 |
| E (este) | ÉXITO ✓ | 2 420 | 0 |
| N (norte) | ÉXITO ✓ | 2 928 | 0 |
| NW (noroeste) | ÉXITO ✓ | 2 503 | 0 |
| SE (sureste) | ÉXITO ✓ | 1 699 | 0 |

Condiciones: R1 = (−1.725, −1.475), R2 = (−1.750, 1.850), carga en el origen, MAX\_STEPS = 4 000, velocidad de empuje y aproximación = 2.0 m/s.

---

## 7. Dificultades encontradas y soluciones

### 7.1 Dirección N — robot atascado en navegación

**Síntoma**: en el escenario de *rally* hacia el norte, el planificador asigna NE+NW (óptimo geométricamente). Sin embargo, R2 parte de (−1.75, 1.85) y debe recorrer ~3 m para alcanzar la posición de aproximación NW, mientras R1 llega en ~250 pasos. R2 nunca convergía en 4 000 pasos.

**Causa raíz compuesta** (tres errores independientes):

1. **Error en `wp_idx`**: `update_assignment()` reiniciaba `wp_idx = 0` en cada replanificación (cada 10 pasos), impidiendo que R2 acumulara progreso a lo largo de su ruta. El robot reiniciaba su índice de *waypoint* antes de alcanzar el siguiente.

2. **Punto ciego en detección de estancamiento**: la EMA de progreso solo se actualizaba cuando `both_pushing = True`. Si R2 permanecía en NAVIGATE, el contador de estancamiento nunca se incrementaba y no se activaba ninguna recuperación.

3. **Peso de navegación insuficiente**: el peso original (0.08) penalizaba escasamente las rutas largas. El planificador elegía configuraciones geométricamente óptimas pero navigationalmente inviables.

**Soluciones aplicadas**:
- El reinicio de `wp_idx` ahora solo ocurre cuando la cara asignada cambia.
- Se añadió `NAV_TIMEOUT = 400` pasos: si el robot no alcanza la posición de aproximación en ese tiempo, se lanza un reinicio sin llamar a `robot.stop()`.
- El peso de navegación se aumentó de 0.08 a 0.30, normalizando el coste respecto a los 3 m de alcance típico de la escena.

### 7.2 Artefacto de congelación de física (*robot.stop()*)

**Síntoma**: tras un reinicio por NAV\_TIMEOUT, el robot quedaba casi inmóvil. La velocidad de cierre caía de 0.038 m/paso a 0.0003 m/paso; la velocidad lineal, de 2.0 a 0.007 m/s. Los reinicios posteriores no recuperaban la movilidad.

**Causa raíz**: llamar a `robot.stop()` (velocidad de ruedas = 0) en CoppeliaSim con el motor de física Bullet activo congela el cuerpo rígido en un estado casi estático que persiste aunque se apliquen nuevas velocidades. Es un comportamiento del simulador, no del código de control.

**Solución**: eliminar toda llamada a `robot.stop()` dentro de `reset()`. El siguiente `drive_to()` redirige el robot sin activar el artefacto. El robot mantiene su velocidad actual durante el reinicio, lo cual es físicamente coherente y no introduce discontinuidades.

### 7.3 Bloqueo estático — direcciones NW y SE

**Síntoma**: ambos robots bloqueados físicamente contra la carga; la EMA de progreso cae a cero; el reinicio por estancamiento se dispara, pero al limpiar todos los bloqueos de asignación, el planificador oscila entre configuraciones equivalentes (70–100 cambios de cara por cada 100 pasos), impidiendo cualquier recuperación efectiva.

**Causa raíz**: sin restricción en la puerta de detección de estancamiento, el contador `_contact_stall_ctr` se incrementaba incluso durante el empuje individual (un solo robot empujando una carga de 20 kg naturalmente se mueve despacio). Además, limpiar `keep_locked=False` en el reinicio destruía la histéresis y permitía que el planificador oscilara libremente.

**Solución**:
- La puerta de detección requiere ahora `in_contact AND both_pushing`: un robot en solitario presionando la carga no está bloqueado.
- El reinicio se llama con `keep_locked=True`, preservando las asignaciones de cara actuales y manteniendo la histéresis.
- Tras `CONTACT_STALL_THR = 30` ticks, el robot entra en BACKOFF (retroceso controlado) y regresa a NAVIGATE para reintroducir el ángulo de aproximación.

### 7.4 Degradación del balance de fuerzas

**Síntoma**: el ratio $f_0/f_1$ disminuye durante la ejecución (de 0.50 a 0.17 en el escenario NE+SE).

**Causa raíz**: las 8 direcciones son fijas en el marco del mundo. A medida que la carga se desplaza, los ángulos relativos entre la dirección de empuje y $\mathbf{F}_{des}$ cambian, degradando la solución NNLS.

**Estado**: limitación estructural conocida, no resuelta en el alcance del proyecto. La solución propuesta es un *rally-frame dynamic basis*: calcular las direcciones de aproximación como ±45° desde $\mathbf{F}_{des}$ en cada paso, garantizando que los vectores de empuje sean siempre simétricos respecto a la dirección deseada.

### 7.5 Guardarraíl de aproximación en el lado del rally

**Síntoma**: en algunos escenarios, el planificador asignaba posiciones de aproximación que implicaban empujar la carga en dirección opuesta al *rally point*.

**Causa raíz**: con 8 direcciones, algunas caras tienen su posición de aproximación en el mismo lado que el destino. Por ejemplo, para un rally al noreste, la cara S tiene su posición de aproximación al norte de la carga — el robot empujaría hacia el norte, pero la carga ya está al sur del punto de extracción, alejándola.

**Solución**: filtro geométrico — se rechaza cualquier asignación cuya posición de aproximación tenga proyección positiva sobre $\mathbf{F}_{des}$ (supera un umbral de 0.1). Esto elimina el sesgo de supervivencia de caras geométricamente inviables.

---

## 8. Limitaciones conocidas

1. **Degeneración de base**: las 8 direcciones fijas ofrecen balance óptimo en $t=0$ pero se degradan a medida que la carga se aproxima al destino. Requiere un marco dinámico alineado con $\mathbf{F}_{des}$.

2. **Espacio de acción discreto**: con solo 8 direcciones posibles, existen configuraciones de rally donde ningún par proporciona ratio de balance superior a 0.50. Ángulos continuos de aproximación eliminarían esta limitación.

3. **Inestabilidad de reasignación cerca del objetivo**: cuando la carga está próxima al *rally point*, pequeñas perturbaciones en posición alteran el orden de puntuación NNLS, causando reasignaciones frecuentes que envían robots a NAVIGATE en el momento más crítico.

4. **Exploración simulada**: la posición de la carga se obtiene mediante `getObjectPosition` (verdad del terreno). La fase de exploración navega hacia el *rally point* hasta que un sensor detecta la carga, pero no incorpora exploración real de entorno desconocido.

---

## 9. Conclusiones

El proyecto demuestra que la descomposición de fuerzas mediante NNLS es un enfoque eficaz para la coordinación de empuje cooperativo multi-robot. La formalización del problema como minimización de residuo bajo restricción de no negatividad proporciona una solución matemáticamente fundamentada, implementable de forma cerrada sin dependencias externas de optimización.

La principal aportación técnica del proyecto es el diseño de la función de puntuación multi-criterio que equilibra la calidad geométrica de la fuerza resultante con la viabilidad navegacional, evitando que el planificador seleccione configuraciones óptimas en papel pero inaccesibles en la práctica. Los pesos de los términos de coste de navegación, par rotacional y balance fueron ajustados iterativamente a partir del análisis de datos CSV generados por los scripts de diagnóstico.

Las dificultades encontradas durante el desarrollo pusieron de manifiesto la importancia de considerar las interacciones entre el software de control y el motor de física del simulador. En particular, el artefacto de `robot.stop()` —que congela permanentemente la dinámica del cuerpo rígido en Bullet— constituyó una causa oculta de fallos que solo fue identificable mediante el análisis detallado de series temporales de velocidad y tasa de cierre.

El sistema final alcanza el objetivo en los cinco escenarios evaluados, con una reducción del 70.6 % en número de pasos respecto a la primera implementación cooperativa, y una mejora de 3.4 × respecto al planificador de cuatro direcciones.

---

## Referencias

- **CoppeliaSim** — Plataforma de simulación robótica empleada para el desarrollo y la validación.
  https://www.coppeliarobotics.com

- **CoppeliaSim Manual** — Referencia de API, scripting de escena y documentación de sensores.
  https://manual.coppeliarobotics.com

- **CoppeliaSim ZMQ Remote API** — Biblioteca cliente Python oficial (`coppeliasim-zmqremoteapi-client`).
  https://github.com/CoppeliaRobotics/zmqRemoteApi

- **coppeliasim-projects-with-zmqRemoteApi** (yudarw) — Ejemplos prácticos de uso de la ZMQ Remote API que orientaron el diseño de la interfaz de control.
  https://github.com/yudarw/coppeliasim-projects-with-zmqRemoteApi
