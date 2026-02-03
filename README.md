# DQN para Navegación en Robótica Móvil (ROS 2)

Este repositorio contiene la implementación de un sistema de navegación autónoma para un robot móvil utilizando Deep Q-Networks (DQN) en un entorno de simulación basado en ROS 2 y Stage.
El agente es entrenado para desplazarse hacia un objetivo evitando colisiones con obstáculos, aprendiendo una política de control a partir de datos sensoriales (LiDAR y odometría).

## Descripción del Proyecto

El proyecto aborda el problema de navegación autónoma mediante aprendizaje por refuerzo, donde el robot:

Percibe el entorno mediante un sensor LiDAR.

Recibe información relativa al objetivo (distancia y ángulo).

Ejecuta acciones discretas de movimiento (avance y giros).

Aprende a maximizar una función de recompensa que incentiva el progreso hacia el objetivo y penaliza colisiones.

Durante el entrenamiento, los objetivos se generan de forma aleatoria en zonas libres de obstáculos, utilizando el mapa del entorno para evitar posiciones inválidas.
Posteriormente, el sistema es evaluado en una fase de prueba independiente para analizar su desempeño y tasa de éxito.

## Enfoque Metodológico

Algoritmo: Deep Q-Network (DQN)

Red neuronal: MLP (MLPRegressor – scikit-learn)

Entradas del estado:

LiDAR discretizado en bins

Distancia al objetivo

Ángulo relativo al objetivo

Espacio de acciones: 5 acciones discretas (avance, giro y combinaciones)

Entorno: Simulación en Stage con ROS 2

Entrenamiento: Exploración ε-greedy y replay buffer

## Requisitos

Ubuntu 20.04 / 22.04

ROS 2 (Humble / Jazzy)

Python 3.8+

Dependencias Python:

pip install numpy scikit-learn

## Uso del Proyecto
1️⃣ Entrenamiento del agente
ros2 run dqn_project train_node


El agente se entrena en el entorno de simulación generando objetivos aleatorios en zonas válidas del mapa.

2️⃣ Pruebas del modelo entrenado
ros2 run dqn_final test_node /home/grace/Documents/Diplomado_Robotica/16_01_ws/assets/model_final.pkl


Se evalúa el desempeño del agente entrenado midiendo éxito y colisiones.

## Resultados

En las pruebas finales realizadas, el sistema alcanzó un success rate del 60%, con una tasa de colisión del 40% en 10 ejecuciones independientes.
Estos resultados indican que el agente aprendió una política de navegación funcional, aunque aún existen oportunidades de mejora para reducir colisiones en entornos más complejos.

## Anexos

🔗 Repositorio GitHub:
https://github.com/GraceLunaVerdueta/dqn_robotica_movil

## Video de demostración:
(https://drive.google.com/drive/folders/174T2z81lO862zKzThYr7oSV7aSs2ivac?usp=sharing)

## Autores

Grace Luna Verdueta
Daniel Alcazar Salas
Santiago Aguilera Salinas
Sandro Murillo Quispe

## Licencia

Este proyecto se desarrolla con fines académicos.
El uso y modificación del código es libre para investigación y aprendizaje, citando la fuente correspondiente.
