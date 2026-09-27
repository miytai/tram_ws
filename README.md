# Резервная одометрия трамвая по модели — ROS 2 Humble

Рабочее пространство решения кейса хакатона «Резервная одометрия по модели».

```
~/tram_ws
├── src/
│   ├── tram_backup_odometry/        <- решение (пакет ROS 2)
│   └── tram_vehicle_msgs/           <- пакет кастомных сообщений из датасета
├── tools/                           <- офлайн-калибровка и инструменты анализа
├── calib/                           <- промежуточные артефакты калибровки
├── build/ install/ log/             <- colcon
```

## Быстрый старт

```bash
source /opt/ros/humble/setup.bash
cd ~/tram_ws
colcon build --packages-select tram_vehicle_msgs tram_backup_odometry
source install/setup.bash

# нода + воспроизведение записи (то, что нужно жюри)
ros2 launch tram_backup_odometry eval_bag.launch.py \
    bag:=$HOME/Downloads/dataset/data/30618_0e41eac3

# метрики по всему датасету
ros2 run tram_backup_odometry evaluate_bag --all $HOME/Downloads/dataset/data

# тесты
colcon test --packages-select tram_backup_odometry && colcon test-result
```

## Документация

| документ | содержание |
|---|---|
| [`src/tram_backup_odometry/README.md`](src/tram_backup_odometry/README.md) | сборка, запуск, структура пакета, ключевые проектные решения (EN) |
| [`src/tram_backup_odometry/docs/MODEL.md`](src/tram_backup_odometry/docs/MODEL.md) | математическая модель, алгоритм оценки, карта пути и начальная привязка, параметры, ограничения |
| [`src/tram_backup_odometry/docs/RESULTS.md`](src/tram_backup_odometry/docs/RESULTS.md) | методика и результаты измерений, абляции, графики |
| [`src/tram_backup_odometry/docs/CALIBRATION.md`](src/tram_backup_odometry/docs/CALIBRATION.md) | офлайн-калибровка: что, чем и как измерено |
| `src/tram_backup_odometry/docs/data/*.csv` | таблицы метрик по каждому прогону |

## Итоговые измерения (87 прогонов с GNSS-эталоном)

| метрика | медиана | p90 |
|---|---|---|
| скорость, RMSE | **0.093 м/с** | 0.401 м/с |
| скорость, bias | **−0.011 м/с** | −0.002 м/с |
| позиция, RMSE | **107 м** | 5672 м |
| накопленный дрейф | **0.17 %** пути | 4.84 % |
| частота публикации | **49.7 Гц** | — |
| задержка вход → выход | **8.9 мс** (p95 19 мс) | 30 мс |
| CPU / RAM | 10.9 % ядра / 61 МБ | — |

## Три находки, определившие решение

1. **Канал скорости колёс записан в км/ч**, хотя комментарий в сообщении
   утверждает м/с: МНК по всем прогонам даёт отношение 1.000207 к скорости
   GNSS.  Без перевода ошибка скорости была бы 19 м/с вместо 0.093 м/с.
2. **Канал положения ручки содержит одиночные сбои** (0.118 % отсчётов) —
   устраняются медианным фильтром окна 5.
3. **Направление движения из разрешённых сигналов не выводится** (у трамвая на
   рельсах нет измеримого курса), а минимальный радиус кривизны трассы депо —
   14 м.  Поэтому позиция публикуется в координатах карты пути (pathgraph),
   что кейс прямо допускает, с начальной привязкой по GNSS внутри
   разрешённого окна.
