### Как кользоваться?

1. Выгрузите `file.csv` в папку `data`. Файл должен имет примерно следующий вид:

```csv
shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint
            0.9453,             -1.0986,    -0.29558,      -0.28334,      -0.20127,        1.9267
           0.94531,             -1.0986,    -0.29561,      -0.28334,      -0.20126,        1.9267
           0.94536,             -1.0985,    -0.29569,      -0.28336,      -0.20122,        1.9268
           0.94543,             -1.0985,    -0.29581,      -0.28339,      -0.20116,        1.9269
           0.94554,             -1.0984,    -0.29599,      -0.28344,      -0.20107,         1.927
```

2. Запустите `MoveIt`, а именно `demo.launch` фашего робота, например.

```bash
roslaunch ur10_moveit_config demo.launch
```

3. Запустите проверку коллизий с указание пути к файлу с траекторией **отностиельно пакета moveit_collision_inspector**. Например:

```bash
roslaunch moveit_collision_inspector start.launch file_path:=data/t6_trj_pos.csv
```