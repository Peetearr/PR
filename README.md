# Поиск выхода из лабиринта

**Цель работы:** Реализовать алгоритм управления мобильным роботом с дифференциальным приводом для выхода из лабиринта. <br />
**Задачи:**
 
  - Моделирование окружения (лабиринта) и робота.
  - Выбор и реализация алгоритма планирования поведения.
  - Реализация программы управления на языке Python.<br />

## Моделирование робота

В данной работе используется робот с дифференциальной схемой управления колесами. Схема робота приведена на рисунке 1 и 2.
![Рисунок 1](./imgs/robot_scheme.png)<br />
![Рисунок 2](./imgs/robot.png)<br />
Робот имеет следующие важные геометрические параметры: 

  - Ширирна колеи: 200 мм.
  - Радиус колеса: 100 мм.

Для определения ориентации робота на нем установлен гироскоп и 6 дальномеров: вперед, назад, вправо, влево, вправо_45, влево_45.

## Моделирование окружение

Общий вид окружения представлен на на рисунке 3. <br />

![Рисунок 3](./imgs/lab_scheme.png)<br />
Робот появляется в клетке старта, каждая клетка имеет размер 500 мм. Целью робота является достичь клетки финиша. Между клетками могут находится стенки.
Для данной работы используется два лабиринта представленные на рисунке 4 и 5.
![Рисунок 4](./imgs/sim1.png)<br />
![Рисунок 5](./imgs/sim2.png)<br />

## Алгоритм планирования поведения

Для задачи поиска выхода из лабиринта можно использовать следующие алгоритмы:

  - Алгоритм правой стены;
  - Поиск в глубину;
  - Поиск в ширирну;
  - Flood fill.
Для данной задачи был выбран алгоритм правой стены, поскольку остальные алгоритмы требуют построения карты окружения для последующего поиска пути, что трудно реализовать только по дальномерам. Идея алгоритма заключается в следования вдоль правой стены до момента поиска. Проблемой алгоритма является неоптимальность процесса поиска и возможность зацикливания алгоритма при сналичии несвязанных стен. Общий вид реализуемой программы приведен ниже:
![Схема](./imgs/scheme.png)

<br /> Определение наличия стены спереди осуществляется передним дальномером. Справа и слева аналогично левым и правым и 45 градусными дальномерами.

## Запуск программы

Some diverse grasps on the objects from DexGraspNet:

![QualitativeResults](./images/qualitative_results.png)

Our synthesis method can be applied to other robotic dexterous hands and the human hand. We provide the complete synthesis pipelines for Allegro and MANO in branches `allegro` and `mano` of this repo. Here are some results: 

![MultiHands](./images/multi_hands.png)

## Overview

This repository provides:

- Simple tools for visualizing grasp data.
- Asset processing for object models. See folder `asset_process`.
- Grasp generation. See folder `grasp_generation`.
  - We also updated code for
    - MANO grasp generation
    - Allegro grasp generation
    - ShadowHand grasp generation for objects on the table
  - See other branches for more information [TODO: update documents].

Our working file structure is as:

```bash
DexGraspNet
+-- asset_process
+-- grasp_generation
+-- data
|  +-- meshdata  # Linked to the output folder of asset processing.
|  +-- experiments  # Linked to a folder in the data disk. Small-scale experimental results go here.
|  +-- graspdata  # Linked to a folder in the data disk. Large-scale generated grasps go here, waiting for grasp validation.
|  +-- dataset  # Linked to a folder in the data disk. Validated results go here.
+-- thirdparty
|  +-- pytorch_kinematics
|  +-- CoACD
|  +-- ManifoldPlus
|  +-- TorchSDF
```

## Quick Example

```bash
conda create -n your_env python=3.7
conda activate your_env

# for quick example, cpu version is OK.
conda install pytorch cpuonly -c pytorch
conda install ipykernel
conda install transforms3d
conda install trimesh
pip install pyyaml
pip install lxml

cd thirdparty/pytorch_kinematics
pip install -e .
```

Then you can run `grasp_generation/quick_example.ipynb`.

For the full DexGraspNet dataset, go to our [project page](https://pku-epic.github.io/DexGraspNet/) for download links. Decompress dowloaded packages and link (or move) them to corresponding path in `data`.
